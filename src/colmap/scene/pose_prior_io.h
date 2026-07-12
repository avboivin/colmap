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

#pragma once

#include "colmap/geometry/pose_prior.h"
#include "colmap/scene/database.h"

#include <filesystem>
#include <string>
#include <vector>

namespace colmap {

struct PosePriorImportStats {
  size_t rows_parsed = 0;
  size_t matched = 0;
  size_t with_position = 0;
  size_t with_gravity = 0;
  size_t with_rotation = 0;
  size_t skipped_unknown = 0;
  size_t rejected_invalid = 0;
  size_t updated = 0;
  size_t inserted = 0;
};

// Parse a pose-prior CSV. Returns false on fatal format errors (bad header,
// mixed coordinate systems). Individual row validation failures increment
// rejected_invalid and are skipped.
//
// CSV columns (header required, order fixed):
//   image_name,coord_system,px,py,pz,
//   cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,
//   gx,gy,gz,qw,qx,qy,qz,
//   rcov_xx,rcov_xy,rcov_xz,rcov_yy,rcov_yz,rcov_zz
//
// Empty cell or "nan" = field absent. Covariance / quaternion groups must be
// all-present or all-absent. Quaternion in CSV is w-first; stored as Eigen
// coeffs (x,y,z,w).
bool ParsePosePriorCsv(const std::filesystem::path& csv_path,
                       const std::vector<Image>& images,
                       std::vector<PosePrior>* pose_priors,
                       PosePriorImportStats* stats,
                       std::string* error_message);

// Import pose priors from CSV into the database. Upserts on existing
// (corr_data_id, corr_sensor_id, corr_sensor_type). If clear_existing, wipes
// the pose_priors table first. If dry_run, parses and validates only.
bool ImportPosePriorsFromCsv(Database* database,
                             const std::filesystem::path& csv_path,
                             bool clear_existing,
                             bool dry_run,
                             PosePriorImportStats* stats,
                             std::string* error_message);

}  // namespace colmap
