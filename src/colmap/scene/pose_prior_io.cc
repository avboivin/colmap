// Copyright (c), ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "colmap/scene/pose_prior_io.h"

#include "colmap/scene/image.h"
#include "colmap/util/logging.h"
#include "colmap/util/misc.h"
#include "colmap/util/string.h"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace colmap {
namespace {

constexpr const char* kExpectedHeader =
    "image_name,coord_system,px,py,pz,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,"
    "gx,gy,gz,qw,qx,qy,qz,rcov_xx,rcov_xy,rcov_xz,rcov_yy,rcov_yz,rcov_zz";

std::string Trim(const std::string& s) {
  const size_t start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const size_t end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(Trim(field));
  }
  return fields;
}

bool IsAbsent(const std::string& s) {
  if (s.empty()) {
    return true;
  }
  std::string lower = s;
  StringToLower(&lower);
  return lower == "nan";
}

bool ParseDouble(const std::string& s, double* value) {
  if (IsAbsent(s)) {
    return false;
  }
  try {
    size_t idx = 0;
    *value = std::stod(s, &idx);
    return idx == s.size() && std::isfinite(*value);
  } catch (...) {
    return false;
  }
}

bool GroupAllPresentOrAbsent(const std::vector<std::string>& fields,
                             size_t begin,
                             size_t count,
                             bool* all_present) {
  size_t present = 0;
  for (size_t i = 0; i < count; ++i) {
    if (!IsAbsent(fields[begin + i])) {
      ++present;
    }
  }
  if (present == 0) {
    *all_present = false;
    return true;
  }
  if (present == count) {
    *all_present = true;
    return true;
  }
  return false;
}

bool IsSymmetricPsd(const Eigen::Matrix3d& m) {
  if (!m.isApprox(m.transpose(), 1e-9)) {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(m);
  return solver.eigenvalues().minCoeff() >= -1e-9;
}

PosePrior::CoordinateSystem ParseCoordSystem(const std::string& s,
                                             bool* ok) {
  std::string lower = s;
  StringToLower(&lower);
  if (lower == "wgs84") {
    *ok = true;
    return PosePrior::CoordinateSystem::WGS84;
  }
  if (lower == "cartesian") {
    *ok = true;
    return PosePrior::CoordinateSystem::CARTESIAN;
  }
  *ok = false;
  return PosePrior::CoordinateSystem::UNDEFINED;
}

}  // namespace

bool ParsePosePriorCsv(const std::filesystem::path& csv_path,
                       const std::vector<Image>& images,
                       std::vector<PosePrior>* pose_priors,
                       PosePriorImportStats* stats,
                       std::string* error_message) {
  THROW_CHECK_NOTNULL(pose_priors);
  THROW_CHECK_NOTNULL(stats);
  *stats = PosePriorImportStats{};
  pose_priors->clear();

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    if (error_message) {
      *error_message = "Failed to open CSV: " + csv_path.string();
    }
    return false;
  }

  std::string header_line;
  if (!std::getline(file, header_line)) {
    if (error_message) {
      *error_message = "CSV is empty";
    }
    return false;
  }
  header_line = Trim(header_line);
  StringToLower(&header_line);
  std::string expected = kExpectedHeader;
  if (header_line != expected) {
    if (error_message) {
      *error_message =
          "Unexpected CSV header. Expected exact column order:\n" +
          std::string(kExpectedHeader);
    }
    return false;
  }

  std::unordered_map<std::string, const Image*> name_to_image;
  name_to_image.reserve(images.size());
  for (const auto& image : images) {
    name_to_image.emplace(image.Name(), &image);
  }

  std::unordered_set<std::string> seen_names;
  std::optional<PosePrior::CoordinateSystem> coord_system;

  std::string line;
  while (std::getline(file, line)) {
    line = Trim(line);
    if (line.empty()) {
      continue;
    }
    ++stats->rows_parsed;

    std::vector<std::string> fields = SplitCsvLine(line);
    // SplitCsvLine may over-count trailing commas; require at least 24 fields.
    if (fields.size() < 24) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Skipping row with " << fields.size()
                   << " fields (need 24): " << line;
      continue;
    }
    // Ignore extra trailing empty fields from a trailing comma.
    if (fields.size() > 24) {
      bool only_empty_extra = true;
      for (size_t i = 24; i < fields.size(); ++i) {
        if (!fields[i].empty()) {
          only_empty_extra = false;
          break;
        }
      }
      if (!only_empty_extra) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Skipping row with too many fields: " << line;
        continue;
      }
      fields.resize(24);
    }

    const std::string& image_name = fields[0];
    if (image_name.empty()) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Skipping row with empty image_name";
      continue;
    }
    if (!seen_names.insert(image_name).second) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Skipping duplicate image_name: " << image_name;
      continue;
    }

    const auto image_it = name_to_image.find(image_name);
    if (image_it == name_to_image.end()) {
      ++stats->skipped_unknown;
      LOG(WARNING) << "Unknown image_name (not in database): " << image_name;
      continue;
    }

    bool coord_ok = false;
    const PosePrior::CoordinateSystem row_coord =
        ParseCoordSystem(fields[1], &coord_ok);
    if (!coord_ok) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Invalid coord_system for " << image_name << ": "
                   << fields[1];
      continue;
    }
    if (!coord_system.has_value()) {
      coord_system = row_coord;
    } else if (coord_system.value() != row_coord) {
      if (error_message) {
        *error_message =
            "Mixed coordinate systems in CSV; all rows must use the same "
            "coord_system (WGS84 or CARTESIAN)";
      }
      return false;
    }

    PosePrior prior;
    prior.corr_data_id = image_it->second->DataId();
    prior.coordinate_system = row_coord;

    bool position_present = false;
    if (!GroupAllPresentOrAbsent(fields, 2, 3, &position_present)) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Partial position for " << image_name;
      continue;
    }
    if (position_present) {
      Eigen::Vector3d position;
      if (!ParseDouble(fields[2], &position.x()) ||
          !ParseDouble(fields[3], &position.y()) ||
          !ParseDouble(fields[4], &position.z())) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Invalid position for " << image_name;
        continue;
      }
      if (row_coord == PosePrior::CoordinateSystem::WGS84) {
        if (position.x() < -90.0 || position.x() > 90.0 ||
            position.y() < -180.0 || position.y() > 180.0) {
          ++stats->rejected_invalid;
          LOG(WARNING) << "WGS84 lat/lon out of range for " << image_name;
          continue;
        }
      }
      prior.position = position;
    }

    bool cov_present = false;
    if (!GroupAllPresentOrAbsent(fields, 5, 6, &cov_present)) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Partial position covariance for " << image_name;
      continue;
    }
    if (cov_present) {
      double xx, xy, xz, yy, yz, zz;
      if (!ParseDouble(fields[5], &xx) || !ParseDouble(fields[6], &xy) ||
          !ParseDouble(fields[7], &xz) || !ParseDouble(fields[8], &yy) ||
          !ParseDouble(fields[9], &yz) || !ParseDouble(fields[10], &zz)) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Invalid position covariance for " << image_name;
        continue;
      }
      Eigen::Matrix3d cov;
      cov << xx, xy, xz, xy, yy, yz, xz, yz, zz;
      if (!IsSymmetricPsd(cov)) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Position covariance not PSD for " << image_name;
        continue;
      }
      prior.position_covariance = cov;
    }

    bool gravity_present = false;
    if (!GroupAllPresentOrAbsent(fields, 11, 3, &gravity_present)) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Partial gravity for " << image_name;
      continue;
    }
    if (gravity_present) {
      Eigen::Vector3d gravity;
      if (!ParseDouble(fields[11], &gravity.x()) ||
          !ParseDouble(fields[12], &gravity.y()) ||
          !ParseDouble(fields[13], &gravity.z())) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Invalid gravity for " << image_name;
        continue;
      }
      const double norm = gravity.norm();
      if (norm < 1e-12) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Zero gravity for " << image_name;
        continue;
      }
      if (std::abs(norm - 1.0) > 0.01) {
        LOG(WARNING) << "Gravity norm " << norm << " for " << image_name
                     << " (expected ~1); normalizing";
      }
      prior.gravity = gravity.normalized();
    }

    bool rotation_present = false;
    if (!GroupAllPresentOrAbsent(fields, 14, 4, &rotation_present)) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Partial rotation quaternion for " << image_name;
      continue;
    }
    if (rotation_present) {
      double qw, qx, qy, qz;
      if (!ParseDouble(fields[14], &qw) || !ParseDouble(fields[15], &qx) ||
          !ParseDouble(fields[16], &qy) || !ParseDouble(fields[17], &qz)) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Invalid rotation for " << image_name;
        continue;
      }
      Eigen::Quaterniond q(qw, qx, qy, qz);
      if (std::abs(q.norm() - 1.0) > 0.001) {
        LOG(WARNING) << "Quaternion norm " << q.norm() << " for " << image_name
                     << " (expected ~1); normalizing";
      }
      prior.rotation = q.normalized();
    }

    bool rcov_present = false;
    if (!GroupAllPresentOrAbsent(fields, 18, 6, &rcov_present)) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "Partial rotation covariance for " << image_name;
      continue;
    }
    if (rcov_present) {
      double xx, xy, xz, yy, yz, zz;
      if (!ParseDouble(fields[18], &xx) || !ParseDouble(fields[19], &xy) ||
          !ParseDouble(fields[20], &xz) || !ParseDouble(fields[21], &yy) ||
          !ParseDouble(fields[22], &yz) || !ParseDouble(fields[23], &zz)) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Invalid rotation covariance for " << image_name;
        continue;
      }
      Eigen::Matrix3d cov;
      cov << xx, xy, xz, xy, yy, yz, xz, yz, zz;
      if (!IsSymmetricPsd(cov)) {
        ++stats->rejected_invalid;
        LOG(WARNING) << "Rotation covariance not PSD for " << image_name;
        continue;
      }
      prior.rotation_covariance = cov;
    }

    if (!prior.HasPosition() && !prior.HasGravity() && !prior.HasRotation()) {
      ++stats->rejected_invalid;
      LOG(WARNING) << "No prior fields present for " << image_name;
      continue;
    }

    ++stats->matched;
    if (prior.HasPosition()) {
      ++stats->with_position;
    }
    if (prior.HasGravity()) {
      ++stats->with_gravity;
    }
    if (prior.HasRotation()) {
      ++stats->with_rotation;
    }
    pose_priors->push_back(prior);
  }

  return true;
}

bool ImportPosePriorsFromCsv(Database* database,
                             const std::filesystem::path& csv_path,
                             bool clear_existing,
                             bool dry_run,
                             PosePriorImportStats* stats,
                             std::string* error_message) {
  THROW_CHECK_NOTNULL(database);
  THROW_CHECK_NOTNULL(stats);

  const std::vector<Image> images = database->ReadAllImages();
  std::vector<PosePrior> pose_priors;
  if (!ParsePosePriorCsv(
          csv_path, images, &pose_priors, stats, error_message)) {
    return false;
  }

  if (dry_run) {
    return true;
  }

  DatabaseTransaction transaction(database);
  if (clear_existing) {
    database->ClearPosePriors();
  }

  // Index existing priors by corr_data_id for upsert.
  std::unordered_map<data_t, PosePrior> existing;
  if (!clear_existing) {
    for (auto& prior : database->ReadAllPosePriors()) {
      existing.emplace(prior.corr_data_id, std::move(prior));
    }
  }

  for (PosePrior& prior : pose_priors) {
    const auto it = existing.find(prior.corr_data_id);
    if (it != existing.end()) {
      prior.pose_prior_id = it->second.pose_prior_id;
      database->UpdatePosePrior(prior);
      ++stats->updated;
    } else {
      prior.pose_prior_id = database->WritePosePrior(prior);
      ++stats->inserted;
    }
  }

  return true;
}

}  // namespace colmap
