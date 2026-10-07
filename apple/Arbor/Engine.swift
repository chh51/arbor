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

import ArborBridge

/// Calls into ArborCore. `resolveOversegmentation`, `extractTreeContext`, and
/// `qsfSegmentSemantic` stay unconnected: their C++ is compiled only for the R package.
enum Engine {
    static func notConnected(_ name: String) -> ArborError {
        .engineNotConnected(name)
    }

    static func segmentGround(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        if cloud.classification == nil {
            cloud.classification = Array(repeating: 0, count: cloud.count)
        }
        if cloud.hag == nil {
            cloud.hag = Array(repeating: 0, count: cloud.count)
        }
        let binding = CloudBinding(cloud)
        var out = ArborBridgeAttributes()
        defer { arbor_bridge_attributes_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withEngineParams(params) { paramsPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_segment_ground(cloudPointer, paramsPointer, &out, &error)
                try check(code, error)
            }
        }
        try apply(&out, to: cloud)
        return cloud
    }

    static func segmentSemantic(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        cloud.userData = Array(repeating: ARBORTREE, count: cloud.count)
        cloud.passage = Array(repeating: 0, count: cloud.count)
        cloud.foliage = Array(repeating: 1, count: cloud.count)
        let binding = CloudBinding(cloud)
        var out = ArborBridgeAttributes()
        defer { arbor_bridge_attributes_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withEngineParams(params) { paramsPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_segment_semantic(cloudPointer, paramsPointer, &out, &error)
                try check(code, error)
            }
        }
        try apply(&out, to: cloud)
        return cloud
    }

    static func segmentInstance(_ cloud: PointCloud, _ seeds: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        cloud.treeID = Array(repeating: 0, count: cloud.count)
        let binding = CloudBinding(cloud)
        let seedBinding = CloudBinding(seeds)
        var out = ArborBridgeAttributes()
        defer { arbor_bridge_attributes_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withUnsafePointer(to: &seedBinding.raw) { seedPointer in
                try withEngineParams(params) { paramsPointer in
                    var error: UnsafeMutablePointer<CChar>?
                    let code = arbor_bridge_segment_instance(cloudPointer, seedPointer, paramsPointer, &out, &error)
                    try check(code, error)
                }
            }
        }
        try apply(&out, to: cloud)
        return cloud
    }

    static func findSeeds(_ cloud: PointCloud, _ params: ArborParameters) throws -> PointCloud {
        let binding = CloudBinding(cloud)
        var out = ArborBridgeSeedCloud()
        defer { arbor_bridge_seeds_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withEngineParams(params) { paramsPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_find_seeds(cloudPointer, paramsPointer, &out, &error)
                try check(code, error)
            }
        }
        return makeSeeds(out, crs: cloud.crs)
    }

    static func homogeneization(_ cloud: PointCloud, res: Double) throws -> PointCloud {
        let binding = CloudBinding(cloud)
        var keep = [UInt8](repeating: 0, count: cloud.count)
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try keep.withUnsafeMutableBufferPointer { buffer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_homogeneization(cloudPointer, res, buffer.baseAddress, &error)
                try check(code, error)
            }
        }
        return cloud.selecting(keep.map { $0 != 0 })
    }

    static func anisotropy(_ cloud: PointCloud, k: Int) throws -> PointCloud {
        let binding = CloudBinding(cloud)
        var scores = [Float](repeating: 0, count: cloud.count)
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try scores.withUnsafeMutableBufferPointer { buffer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_wood_likelihood(cloudPointer, Int32(k), buffer.baseAddress, &error)
                try check(code, error)
            }
        }
        cloud.pwood = scores
        return cloud
    }

    static func resolveOversegmentation(_ cloud: PointCloud) throws -> PointCloud {
        throw notConnected("resolveOversegmentation")
    }

    static func colorize(_ cloud: PointCloud, darkenFoliage: Bool) throws -> PointCloud {
        let count = cloud.count
        cloud.red = Array(repeating: 150, count: count)
        cloud.green = Array(repeating: 150, count: count)
        cloud.blue = Array(repeating: 150, count: count)
        let binding = CloudBinding(cloud)
        var out = ArborBridgeAttributes()
        defer { arbor_bridge_attributes_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_colorize(cloudPointer, darkenFoliage ? 1 : 0, &out, &error)
            try check(code, error)
        }
        try apply(&out, to: cloud)
        return cloud
    }

    static func qsm(_ tree: PointCloud, _ params: ArborParameters) throws -> QSM {
        let binding = CloudBinding(tree)
        var out = ArborBridgeQSM()
        defer { arbor_bridge_qsm_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withEngineParams(params) { paramsPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_qsm(cloudPointer, paramsPointer, &out, &error)
                try check(code, error)
            }
        }
        return makeQSM(out)
    }

    static func qsf(_ cloud: PointCloud, minHeight: Double, params: ArborParameters) throws -> QSF {
        if cloud.userData == nil {
            cloud.userData = Array(repeating: ARBORTREE, count: cloud.count)
        }
        let binding = CloudBinding(cloud)
        var out = ArborBridgeQSF()
        defer { arbor_bridge_qsf_free(&out) }
        try withUnsafePointer(to: &binding.raw) { cloudPointer in
            try withEngineParams(params) { paramsPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_qsf(cloudPointer, minHeight, paramsPointer, &out, &error)
                try check(code, error)
            }
        }
        return makeQSF(out)
    }

    static func qsmDbh(_ model: QSM, breastHeight: Double) throws -> DBH {
        let binding = QSMBinding(model)
        var out = ArborBridgeDBH()
        try withUnsafePointer(to: &binding.raw) { modelPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_qsm_dbh(modelPointer, breastHeight, &out, &error)
            try check(code, error)
        }
        return makeDBH(out)
    }

    static func qsmStem(_ model: QSM) throws -> QSM {
        let binding = QSMBinding(model)
        var out = ArborBridgeQSM()
        defer { arbor_bridge_qsm_free(&out) }
        try withUnsafePointer(to: &binding.raw) { modelPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_qsm_stem(modelPointer, &out, &error)
            try check(code, error)
        }
        let result = makeQSM(out)
        if result.crs.isEmpty { result.crs = model.crs }
        return result
    }

    static func qsmMerchantable(_ model: QSM, radius: Double, length: Double) throws -> QSM {
        let binding = QSMBinding(model)
        var out = ArborBridgeQSM()
        defer { arbor_bridge_qsm_free(&out) }
        try withUnsafePointer(to: &binding.raw) { modelPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_qsm_merchantable(modelPointer, radius, length, &out, &error)
            try check(code, error)
        }
        let result = makeQSM(out)
        if result.crs.isEmpty { result.crs = model.crs }
        return result
    }

    static func readQSM(_ path: String) throws -> QSM {
        var out = ArborBridgeQSM()
        defer { arbor_bridge_qsm_free(&out) }
        try path.withCString { pathPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_qsm_read(pathPointer, &out, &error)
            try check(code, error)
        }
        return makeQSM(out)
    }

    static func writeQSM(_ model: QSM, to path: String, binary: Bool) throws {
        let binding = QSMBinding(model)
        try path.withCString { pathPointer in
            try withUnsafePointer(to: &binding.raw) { modelPointer in
                var error: UnsafeMutablePointer<CChar>?
                let code = arbor_bridge_qsm_write(modelPointer, pathPointer, binary ? 1 : 0, &error)
                try check(code, error)
            }
        }
    }

    static func readQSF(_ path: String) throws -> QSF {
        var out = ArborBridgeQSF()
        defer { arbor_bridge_qsf_free(&out) }
        try path.withCString { pathPointer in
            var error: UnsafeMutablePointer<CChar>?
            let code = arbor_bridge_qsf_read(pathPointer, &out, &error)
            try check(code, error)
        }
        return makeQSF(out)
    }

    static func writeQSF(_ forest: QSF, to path: String, format: String, binary: Bool) throws {
        let bindings = forest.models.map { QSMBinding($0) }
        var models = bindings.map { $0.raw }
        try path.withCString { pathPointer in
            try format.withCString { formatPointer in
                try models.withUnsafeMutableBufferPointer { buffer in
                    var error: UnsafeMutablePointer<CChar>?
                    let code = arbor_bridge_qsf_write(buffer.baseAddress, buffer.count, pathPointer, formatPointer, binary ? 1 : 0, &error)
                    try check(code, error)
                }
            }
        }
    }

    static func extractTreeContext(_ cloud: PointCloud, treeID: Int, excludeTree: Bool) throws -> PointCloud {
        throw notConnected("extractTreeContext")
    }

    static func qsmDistances(_ forest: QSF, _ cloud: PointCloud) throws -> [(distance: Double, radius: Double)] {
        throw notConnected("qsfSegmentSemantic")
    }
}
