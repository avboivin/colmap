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

#include "colmap/scene/database.h"
#include "colmap/scene/database_sqlite.h"
#include "colmap/util/testing.h"

#include <fstream>

#include <gtest/gtest.h>

namespace colmap {
namespace {

const char* kHeader =
    "image_name,coord_system,px,py,pz,cov_xx,cov_xy,cov_xz,cov_yy,cov_yz,cov_zz,"
    "gx,gy,gz,qw,qx,qy,qz,rcov_xx,rcov_xy,rcov_xz,rcov_yy,rcov_yz,rcov_zz\n";

class PosePriorIoTests : public ::testing::Test {
 protected:
  void SetUp() override {
    database_ = Database::Open(kInMemorySqliteDatabasePath);
    Camera camera;
    camera.camera_id = database_->WriteCamera(camera);
    Image image1;
    image1.SetName("img1.jpg");
    image1.SetCameraId(camera.camera_id);
    image1.SetImageId(database_->WriteImage(image1));
    Image image2;
    image2.SetName("img2.jpg");
    image2.SetCameraId(camera.camera_id);
    image2.SetImageId(database_->WriteImage(image2));
  }

  std::filesystem::path WriteTempCsv(const std::string& body) {
    const std::filesystem::path path = CreateTestDir() / "pose_priors.csv";
    std::ofstream file(path);
    file << kHeader << body;
    return path;
  }

  std::shared_ptr<Database> database_;
};

TEST_F(PosePriorIoTests, ImportCartesianFull) {
  // 24 fields: name, CARTESIAN, pos, cov, gravity, quat(wxyz), rcov
  const auto path = WriteTempCsv(
      "img1.jpg,CARTESIAN,1,2,3,4,0,0,5,0,6,0,1,0,1,0,0,0,0.01,0,0,0.01,0,0.04\n"
      "img2.jpg,CARTESIAN,4,5,6,,,,,,,,,,,,,,,,,,,\n"
      "unknown.jpg,CARTESIAN,7,8,9,,,,,,,,,,,,,,,,,,,\n");

  PosePriorImportStats stats;
  std::string error;
  ASSERT_TRUE(ImportPosePriorsFromCsv(database_.get(),
                                      path,
                                      /*clear_existing=*/false,
                                      /*dry_run=*/false,
                                      &stats,
                                      &error))
      << error;
  EXPECT_EQ(stats.rows_parsed, 3);
  EXPECT_EQ(stats.matched, 2);
  EXPECT_EQ(stats.skipped_unknown, 1);
  EXPECT_EQ(stats.with_position, 2);
  EXPECT_EQ(stats.with_gravity, 1);
  EXPECT_EQ(stats.with_rotation, 1);
  EXPECT_EQ(stats.inserted, 2);

  const std::vector<PosePrior> priors = database_->ReadAllPosePriors();
  ASSERT_EQ(priors.size(), 2);
  PosePrior p1;
  PosePrior p2;
  for (const auto& p : priors) {
    if (p.HasRotation()) {
      p1 = p;
    } else {
      p2 = p;
    }
  }
  EXPECT_TRUE(p1.HasPosition());
  EXPECT_NEAR(p1.position.x(), 1, 1e-9);
  EXPECT_TRUE(p1.HasGravity());
  EXPECT_TRUE(p1.HasRotation());
  EXPECT_NEAR(p1.rotation.w(), 1, 1e-9);
  EXPECT_TRUE(p1.HasRotationCov());
  EXPECT_NEAR(p1.rotation_covariance(2, 2), 0.04, 1e-9);

  EXPECT_TRUE(p2.HasPosition());
  EXPECT_FALSE(p2.HasGravity());
  EXPECT_FALSE(p2.HasRotation());
}

TEST_F(PosePriorIoTests, RejectBadCovarianceAndDuplicate) {
  // Non-PSD position covariance (zz = -1).
  const auto path = WriteTempCsv(
      "img1.jpg,CARTESIAN,1,2,3,1,0,0,1,0,-1,,,,,,,,,,,,,\n"
      "img1.jpg,CARTESIAN,4,5,6,,,,,,,,,,,,,,,,,,,\n");

  PosePriorImportStats stats;
  std::string error;
  ASSERT_TRUE(ImportPosePriorsFromCsv(database_.get(),
                                      path,
                                      /*clear_existing=*/false,
                                      /*dry_run=*/false,
                                      &stats,
                                      &error))
      << error;
  EXPECT_GE(stats.rejected_invalid, 1);
}

TEST_F(PosePriorIoTests, MixedCoordSystemsFails) {
  const auto path = WriteTempCsv(
      "img1.jpg,CARTESIAN,1,2,3,,,,,,,,,,,,,,,,,,,\n"
      "img2.jpg,WGS84,10,20,30,,,,,,,,,,,,,,,,,,,\n");

  PosePriorImportStats stats;
  std::string error;
  EXPECT_FALSE(ImportPosePriorsFromCsv(database_.get(),
                                       path,
                                       /*clear_existing=*/false,
                                       /*dry_run=*/true,
                                       &stats,
                                       &error));
  EXPECT_FALSE(error.empty());
}

TEST_F(PosePriorIoTests, UpsertUpdatesExisting) {
  PosePrior existing;
  existing.corr_data_id = database_->ReadAllImages().front().DataId();
  existing.position = Eigen::Vector3d(9, 9, 9);
  existing.coordinate_system = PosePrior::CoordinateSystem::CARTESIAN;
  existing.gravity = Eigen::Vector3d(0, 1, 0);
  existing.pose_prior_id = database_->WritePosePrior(existing);

  // Position-only CSV row must preserve EXIF/existing gravity.
  const auto path =
      WriteTempCsv("img1.jpg,CARTESIAN,1,2,3,,,,,,,,,,,,,,,,,,,\n");

  PosePriorImportStats stats;
  std::string error;
  ASSERT_TRUE(ImportPosePriorsFromCsv(database_.get(),
                                      path,
                                      /*clear_existing=*/false,
                                      /*dry_run=*/false,
                                      &stats,
                                      &error))
      << error;
  EXPECT_EQ(stats.updated, 1);
  EXPECT_EQ(stats.inserted, 0);
  const PosePrior read = database_->ReadPosePrior(
      existing.pose_prior_id, /*is_deprecated_image_prior=*/false);
  EXPECT_NEAR(read.position.x(), 1, 1e-9);
  EXPECT_TRUE(read.HasGravity());
  EXPECT_NEAR(read.gravity.y(), 1, 1e-9);
}

TEST_F(PosePriorIoTests, RejectCoordSystemConflictWithDb) {
  PosePrior existing;
  existing.corr_data_id = database_->ReadAllImages()[1].DataId();  // img2
  existing.position = Eigen::Vector3d(1, 2, 3);
  existing.coordinate_system = PosePrior::CoordinateSystem::WGS84;
  existing.pose_prior_id = database_->WritePosePrior(existing);

  // CSV only updates img1 as CARTESIAN; img2 stays WGS84 → conflict.
  const auto path =
      WriteTempCsv("img1.jpg,CARTESIAN,1,2,3,,,,,,,,,,,,,,,,,,,\n");

  PosePriorImportStats stats;
  std::string error;
  EXPECT_FALSE(ImportPosePriorsFromCsv(database_.get(),
                                       path,
                                       /*clear_existing=*/false,
                                       /*dry_run=*/true,
                                       &stats,
                                       &error));
  EXPECT_FALSE(error.empty());
}

TEST_F(PosePriorIoTests, DryRunDoesNotWrite) {
  const auto path =
      WriteTempCsv("img1.jpg,CARTESIAN,1,2,3,,,,,,,,,,,,,,,,,,,\n");
  PosePriorImportStats stats;
  std::string error;
  ASSERT_TRUE(ImportPosePriorsFromCsv(database_.get(),
                                      path,
                                      /*clear_existing=*/false,
                                      /*dry_run=*/true,
                                      &stats,
                                      &error))
      << error;
  EXPECT_EQ(stats.matched, 1);
  EXPECT_EQ(database_->NumPosePriors(), 0);
}

}  // namespace
}  // namespace colmap
