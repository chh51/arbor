// @file Engine.swift
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

/// C++ entry points. The public functions call these. They throw until the
/// Swift target is connected to `ArborCore`.
enum Engine {
    static func notConnected(_ name: String) -> ArborError {
        .engineNotConnected(name)
    }

    static func segmentGround(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        throw notConnected("segmentGround")
    }

    static func segmentSemantic(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        throw notConnected("segmentSemantic")
    }

    static func segmentInstance(_ cloud: PointCloud, _ seeds: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        throw notConnected("segmentInstance")
    }

    static func findSeeds(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        throw notConnected("findSeeds")
    }

    static func homogeneization(_ cloud: PointCloud, res: Double) throws -> PointCloud {
        throw notConnected("hybridHomogeneization")
    }

    static func anisotropy(_ cloud: PointCloud, k: Int) throws -> PointCloud {
        throw notConnected("woodLikelihood")
    }

    static func resolveOversegmentation(_ cloud: PointCloud) throws -> PointCloud {
        throw notConnected("resolveOversegmentation")
    }

    static func colorize(_ cloud: PointCloud, darkenFoliage: Bool) throws -> PointCloud {
        throw notConnected("colorizeTrees")
    }

    static func qsm(_ tree: PointCloud, _ params: ArborParameters) throws -> QSM {
        throw notConnected("qsm")
    }

    static func qsf(_ cloud: PointCloud, minHeight: Double, params: ArborParameters) throws -> QSF {
        throw notConnected("qsf")
    }

    static func qsmDbh(_ model: QSM, breastHeight: Double) throws -> DBH {
        throw notConnected("qsmDbh")
    }

    static func qsmStem(_ model: QSM) throws -> QSM {
        throw notConnected("qsmStem")
    }

    static func qsmMerchantable(_ model: QSM, radius: Double, length: Double) throws -> QSM {
        throw notConnected("qsmMerchantable")
    }

    static func readQSM(_ path: String) throws -> QSM {
        throw notConnected("qsmRead")
    }

    static func writeQSM(_ model: QSM, to path: String, binary: Bool) throws {
        throw notConnected("qsmWrite")
    }

    static func readQSF(_ path: String) throws -> QSF {
        throw notConnected("qsfRead")
    }

    static func writeQSF(_ forest: QSF, to path: String, format: String, binary: Bool) throws {
        throw notConnected("qsfWrite")
    }

    static func extractTreeContext(_ cloud: PointCloud, treeID: Int, excludeTree: Bool) throws -> PointCloud {
        throw notConnected("extractTreeContext")
    }

    static func qsmDistances(_ forest: QSF, _ cloud: PointCloud) throws -> [(distance: Double, radius: Double)] {
        throw notConnected("qsfSegmentSemantic")
    }
}
