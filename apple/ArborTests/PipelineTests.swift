// @file PipelineTests.swift
// Project: Arbor
//
// Copyright (C) 2026 Jean-Romain Roussel (r-lidar) <info @ r-lidar.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

import Testing
import Arbor

/// In-memory checks that the Swift API reaches ArborCore. Fixture cases from
/// `tests/testthat/test-pipeline.R` belong in this file once a LAS reader exists.
@Suite(.serialized)
struct PipelineTests {
    @Test func woodLikelihoodWritesAScoreOnEveryPoint() throws {
        let cloud = PointCloud(
            x: [0, 0, 0, 0],
            y: [0, 0, 0, 0],
            z: [0, 1, 2, 3]
        )
        var params = arborParametersDefault
        params.woodLikelihood.k = 2

        let scored = try woodLikelihood(cloud, params: params)

        let pwood = try #require(scored.pwood)
        #expect(pwood.count == 4)
        #expect(pwood.allSatisfy { $0.isFinite })
    }

    @Test func hybridHomogeneizationKeepsOnePointFromASingleVoxel() throws {
        let cloud = PointCloud(
            x: [0, 0.01, 0.02, 0.03],
            y: [0, 0, 0, 0],
            z: [0, 0, 0, 0]
        )

        let thinned = try hybridHomogeneization(cloud, res: 10)

        #expect(cloud.count == 4)
        #expect(thinned.count == 1)
    }
}
