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

#include "colmap/exe/database.h"

#include "colmap/controllers/option_manager.h"
#include "colmap/scene/database.h"
#include "colmap/scene/pose_prior_io.h"
#include "colmap/scene/reconstruction.h"
#include "colmap/scene/rig.h"
#include "colmap/util/file.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace colmap {

int RunDatabaseCleaner(int argc, char** argv) {
  std::string type;

  OptionManager options;
  options.AddRequiredOption(
      "type", &type, "{all, images, features, matches, two_view_geometries}");
  options.AddDatabaseOptions();
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  StringToLower(&type);
  auto database = Database::Open(*options.database_path);

  DatabaseTransaction transaction(database.get());
  if (type == "all") {
    LOG(INFO) << "Clearing all tables";
    database->ClearAllTables();
  } else if (type == "images") {
    LOG(INFO) << "Clearing images and all dependent tables";
    database->ClearImages();
    database->ClearMatches();
    database->ClearTwoViewGeometries();
  } else if (type == "features") {
    LOG(INFO) << "Clearing features, matches, and two-view geometries";
    database->ClearDescriptors();
    database->ClearKeypoints();
    database->ClearMatches();
    database->ClearTwoViewGeometries();
  } else if (type == "matches") {
    LOG(INFO) << "Clearing matches and two-view geometries";
    database->ClearMatches();
    database->ClearTwoViewGeometries();
  } else if (type == "two_view_geometries") {
    LOG(INFO) << "Clearing two-view geometries";
    database->ClearTwoViewGeometries();
  } else {
    LOG(ERROR) << "Invalid cleanup type; no changes in database";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int RunDatabaseCreator(int argc, char** argv) {
  OptionManager options;
  options.AddDatabaseOptions();
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  auto database = Database::Open(*options.database_path);

  return EXIT_SUCCESS;
}

int RunDatabaseMerger(int argc, char** argv) {
  std::filesystem::path database_path1;
  std::filesystem::path database_path2;
  std::filesystem::path merged_database_path;

  OptionManager options;
  options.AddRequiredOption("database_path1", &database_path1);
  options.AddRequiredOption("database_path2", &database_path2);
  options.AddRequiredOption("merged_database_path", &merged_database_path);
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  if (ExistsFile(merged_database_path)) {
    LOG(ERROR) << "Merged database file must not exist.";
    return EXIT_FAILURE;
  }

  auto database1 = Database::Open(database_path1);
  auto database2 = Database::Open(database_path2);
  auto merged_database = Database::Open(merged_database_path);
  Database::Merge(*database1, *database2, merged_database.get());

  return EXIT_SUCCESS;
}

int RunRigConfigurator(int argc, char** argv) {
  std::filesystem::path database_path;
  std::filesystem::path rig_config_path;
  std::filesystem::path input_path;
  std::filesystem::path output_path;

  OptionManager options;
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption("rig_config_path",
                            &rig_config_path,
                            "Rig configuration as a .json file.");
  options.AddDefaultOption("input_path",
                           &input_path,
                           "Optional input reconstruction to automatically "
                           "derive the (average) rig and camera calibrations. "
                           "If not provided, the rig intrinsics and extrinsics "
                           "must be specified in the provided config.");
  options.AddDefaultOption(
      "output_path",
      &output_path,
      "Optional output reconstruction with configured rigs/frames.");
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  std::optional<Reconstruction> reconstruction;
  if (!input_path.empty()) {
    reconstruction = std::make_optional<Reconstruction>();
    reconstruction->Read(input_path);
  }

  auto database = Database::Open(database_path);

  ApplyRigConfig(
      ReadRigConfig(rig_config_path),
      *database,
      reconstruction.has_value() ? &reconstruction.value() : nullptr);

  if (reconstruction.has_value() && !output_path.empty()) {
    reconstruction->Write(output_path);
  }

  return EXIT_SUCCESS;
}

int RunPosePriorImporter(int argc, char** argv) {
  std::filesystem::path database_path;
  std::filesystem::path import_path;
  bool clear_existing = false;
  bool dry_run = false;

  OptionManager options;
  options.AddRequiredOption("database_path", &database_path);
  options.AddRequiredOption(
      "import_path",
      &import_path,
      "CSV file with pose priors (see docs for column format).");
  options.AddDefaultOption("clear_existing",
                           &clear_existing,
                           "If true, clear the pose_priors table before import.");
  options.AddDefaultOption(
      "dry_run", &dry_run, "Parse and validate only; do not write to the database.");
  if (!options.Parse(argc, argv)) {
    return EXIT_FAILURE;
  }

  auto database = Database::Open(database_path);
  PosePriorImportStats stats;
  std::string error_message;
  if (!ImportPosePriorsFromCsv(database.get(),
                               import_path,
                               clear_existing,
                               dry_run,
                               &stats,
                               &error_message)) {
    LOG(ERROR) << error_message;
    return EXIT_FAILURE;
  }

  LOG(INFO) << "Pose prior import"
            << (dry_run ? " (dry run)" : "") << ": parsed=" << stats.rows_parsed
            << " matched=" << stats.matched
            << " position=" << stats.with_position
            << " gravity=" << stats.with_gravity
            << " rotation=" << stats.with_rotation
            << " unknown=" << stats.skipped_unknown
            << " rejected=" << stats.rejected_invalid
            << " inserted=" << stats.inserted << " updated=" << stats.updated;
  return EXIT_SUCCESS;
}

}  // namespace colmap
