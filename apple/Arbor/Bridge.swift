// @file Bridge.swift
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
import Darwin

func check(_ code: Int32, _ error: UnsafeMutablePointer<CChar>?) throws {
    guard code != 0 else { return }
    let message = error.map { String(cString: $0) } ?? "Arbor engine failed"
    if let error {
        arbor_bridge_string_free(error)
    }
    throw ArborError.engineFailed(message)
}

func withEngineParams<R>(_ params: ArborParameters, _ body: (UnsafePointer<ArborBridgeParams>) throws -> R) rethrows -> R {
    try params.pathfinder.anglePenalty.withUnsafeBufferPointer { angles in
        try params.seeds.sliceAt.withUnsafeBufferPointer { slices in
            try params.qsm.allometryName.withCString { name in
                var raw = ArborBridgeParams()
                raw.cut_above_ground = params.global.cutAboveGround
                raw.wood_k = Int32(params.woodLikelihood.k)
                raw.downward = params.pathfinder.downward ? 1 : 0
                raw.graph_k = Int32(params.pathfinder.k)
                raw.k_seed = Int32(params.pathfinder.kSeed)
                raw.decimation = params.pathfinder.decimation
                raw.space_res = params.pathfinder.spaceRes
                raw.max_gap = params.pathfinder.maxGap
                raw.power = params.pathfinder.power
                raw.wood_to_wood = params.pathfinder.woodToWood
                raw.leaf_to_leaf = params.pathfinder.leafToLeaf
                raw.wood_to_leaf = params.pathfinder.woodToLeaf
                raw.angle_penalty = angles.baseAddress
                raw.angle_penalty_count = Int32(angles.count)
                raw.min_passage = Int32(params.semantic.minPassage)
                raw.high_pwood_threshold = params.semantic.highPwoodThreshold
                raw.medium_pwood_threshold = params.semantic.mediumPwoodThreshold
                raw.connected_components_res = params.semantic.connectedComponentsRes
                raw.connected_components_min = Int32(params.semantic.connectedComponentsMin)
                raw.wood_assignation_k = Int32(params.semantic.woodAssignationK)
                raw.wood_assignation_dist = params.semantic.woodAssignationDist
                raw.wood_extra_reasignation_k = Int32(params.semantic.woodExtraReasignationK)
                raw.wood_extra_reasignation_dist = params.semantic.woodExtraReasignationDist
                raw.medium_pwood_sor_k = Int32(params.semantic.mediumPwoodSorK)
                raw.medium_pwood_sor_m = params.semantic.mediumPwoodSorM
                raw.ground_res = params.semantic.groundRes
                raw.slice_at = slices.baseAddress
                raw.slice_at_count = Int32(slices.count)
                raw.slice_thickness = params.seeds.sliceThickness
                raw.seed_min_passage = Int32(params.seeds.minPassage)
                raw.safe_zone = params.seeds.safeZone
                raw.oversegmentation_solver_enabled = params.instance.oversegmentationSolverEnabled ? 1 : 0
                raw.skeleton_node_distance = params.qsm.skeletonNodeDistance
                raw.dbscan_eps_distance = params.qsm.dbscanEpsDistance
                raw.max_d = params.qsm.maxD
                raw.apex_radius = params.qsm.apexRadius
                raw.smooth_steps = Int32(params.qsm.smoothSteps)
                raw.min_measurable_dbh = params.qsm.minMeasurableDBH
                raw.min_measurable_radius = params.qsm.minMeasurableRadius
                raw.broken_detection_enabled = params.qsm.brokenDetectionEnabled ? 1 : 0
                raw.allometry_scale = params.qsm.allometryScale
                raw.allometry_name = name
                return try withUnsafePointer(to: &raw) { try body($0) }
            }
        }
    }
}

final class CloudBinding {
    var raw = ArborBridgeCloud()
    private var x: UnsafeMutablePointer<Double>?
    private var y: UnsafeMutablePointer<Double>?
    private var z: UnsafeMutablePointer<Double>?
    private var classification: UnsafeMutablePointer<Int32>?
    private var hag: UnsafeMutablePointer<Double>?
    private var pwood: UnsafeMutablePointer<Float>?
    private var foliage: UnsafeMutablePointer<Int32>?
    private var passage: UnsafeMutablePointer<Int32>?
    private var userData: UnsafeMutablePointer<Int32>?
    private var treeID: UnsafeMutablePointer<Int32>?
    private var red: UnsafeMutablePointer<UInt16>?
    private var green: UnsafeMutablePointer<UInt16>?
    private var blue: UnsafeMutablePointer<UInt16>?
    private let count: Int

    init(_ cloud: PointCloud) {
        count = cloud.count
        x = bind(cloud.x)
        y = bind(cloud.y)
        z = bind(cloud.z)
        classification = bind(cloud.classification)
        hag = bind(cloud.hag)
        pwood = bind(cloud.pwood)
        foliage = bind(cloud.foliage)
        passage = bind(cloud.passage)
        userData = bind(cloud.userData)
        treeID = bind(cloud.treeID)
        red = bind(cloud.red)
        green = bind(cloud.green)
        blue = bind(cloud.blue)
        raw.count = count
        raw.x = x
        raw.y = y
        raw.z = z
        raw.classification = classification
        raw.hag = hag
        raw.pwood = pwood
        raw.foliage = foliage
        raw.passage = passage
        raw.user_data = userData
        raw.tree_id = treeID
        raw.red = red
        raw.green = green
        raw.blue = blue
    }

    deinit { deallocate() }

    private func bind<T>(_ values: [T]?) -> UnsafeMutablePointer<T>? {
        guard let values, !values.isEmpty else { return nil }
        let pointer = UnsafeMutablePointer<T>.allocate(capacity: values.count)
        pointer.initialize(from: values, count: values.count)
        return pointer
    }

    private func deallocate() {
        release(&x)
        release(&y)
        release(&z)
        release(&classification)
        release(&hag)
        release(&pwood)
        release(&foliage)
        release(&passage)
        release(&userData)
        release(&treeID)
        release(&red)
        release(&green)
        release(&blue)
    }

    private func release<T>(_ pointer: inout UnsafeMutablePointer<T>?) {
        guard let value = pointer else { return }
        value.deinitialize(count: count)
        value.deallocate()
        pointer = nil
    }
}

func apply(_ out: inout ArborBridgeAttributes, to cloud: PointCloud) throws {
    let count = Int(out.count)
    if count != cloud.count {
        throw ArborError.engineFailed("engine returned \(count) points for \(cloud.count)")
    }
    if out.reordered != 0 {
        guard let order = out.order else {
            throw ArborError.engineFailed("reordered point cloud is missing its index")
        }
        try cloud.reorder(by: Array(UnsafeBufferPointer(start: order, count: count)))
    }
    if let values = integers(out.classification_set, out.classification, count) { cloud.classification = values }
    if let values = doubles(out.hag_set, out.hag, count) { cloud.hag = values }
    if let values = integers(out.foliage_set, out.foliage, count) { cloud.foliage = values }
    if let values = integers(out.passage_set, out.passage, count) { cloud.passage = values }
    if let values = integers(out.user_data_set, out.user_data, count) { cloud.userData = values }
    if let values = integers(out.tree_id_set, out.tree_id, count) { cloud.treeID = values }
    if out.rgb_set != 0 {
        cloud.red = words(out.red, count)
        cloud.green = words(out.green, count)
        cloud.blue = words(out.blue, count)
    }
}

func makeSeeds(_ raw: ArborBridgeSeedCloud, crs: String) -> PointCloud {
    let count = Int(raw.count)
    let cloud = PointCloud(x: doubles(1, raw.x, count) ?? [], y: doubles(1, raw.y, count) ?? [], z: doubles(1, raw.z, count) ?? [], crs: crs)
    cloud.treeID = integers(raw.tree_id_set, raw.tree_id, count)
    cloud.foliage = integers(raw.foliage_set, raw.foliage, count)
    cloud.hag = doubles(raw.hag_set, raw.hag, count)
    cloud.pwood = floats(raw.pwood_set, raw.pwood, count)
    cloud.passage = integers(raw.passage_set, raw.passage, count)
    return cloud
}

final class QSMBinding {
    var raw = ArborBridgeQSM()
    private var cylinders: UnsafeMutablePointer<ArborBridgeCylinder>?
    private var cylinderCount = 0
    private var messages: [UnsafeMutablePointer<CChar>?] = []
    private var messagePointers: UnsafeMutablePointer<UnsafeMutablePointer<CChar>?>?
    private var name: UnsafeMutablePointer<CChar>?
    private var crs: UnsafeMutablePointer<CChar>?

    init(_ model: QSM) {
        cylinderCount = model.cylinders.count
        if cylinderCount > 0 {
            let pointer = UnsafeMutablePointer<ArborBridgeCylinder>.allocate(capacity: cylinderCount)
            for (index, cylinder) in model.cylinders.enumerated() {
                pointer[index] = ArborBridgeCylinder(
                    start_x: cylinder.startX,
                    start_y: cylinder.startY,
                    start_z: cylinder.startZ,
                    end_x: cylinder.endX,
                    end_y: cylinder.endY,
                    end_z: cylinder.endZ,
                    cyl_id: cylinder.cylID,
                    parent_id: cylinder.parentID,
                    axis_id: cylinder.axisID,
                    branch_order: cylinder.branchOrder,
                    quality: cylinder.quality,
                    radius: cylinder.radius,
                    dist_to_root: cylinder.distToRoot,
                    subtree_length: cylinder.subtreeLength
                )
            }
            cylinders = pointer
        }
        messages = model.message.map { strdup($0) }
        if !messages.isEmpty {
            let pointer = UnsafeMutablePointer<UnsafeMutablePointer<CChar>?>.allocate(capacity: messages.count)
            for (index, message) in messages.enumerated() {
                pointer[index] = message
            }
            messagePointers = pointer
        }
        if let modelName = model.name {
            name = strdup(modelName)
        }
        crs = strdup(model.crs)
        raw.id = Int32(model.id ?? 0)
        raw.name = name
        raw.crs = crs
        raw.messages = messagePointers
        raw.message_count = messages.count
        raw.cylinders = cylinders
        raw.cylinder_count = cylinderCount
    }

    deinit {
        if let cylinders {
            cylinders.deinitialize(count: cylinderCount)
            cylinders.deallocate()
        }
        if let messagePointers {
            messagePointers.deinitialize(count: messages.count)
            messagePointers.deallocate()
        }
        for message in messages { free(message) }
        free(name)
        free(crs)
    }
}

func makeQSM(_ raw: ArborBridgeQSM) -> QSM {
    let count = Int(raw.cylinder_count)
    var cylinders: [Cylinder] = []
    if count > 0, let pointer = raw.cylinders {
        cylinders.reserveCapacity(count)
        for index in 0..<count {
            let row = pointer[index]
            cylinders.append(Cylinder(
                startX: row.start_x,
                startY: row.start_y,
                startZ: row.start_z,
                endX: row.end_x,
                endY: row.end_y,
                endZ: row.end_z,
                cylID: row.cyl_id,
                parentID: row.parent_id,
                axisID: row.axis_id,
                branchOrder: row.branch_order,
                distToRoot: row.dist_to_root,
                subtreeLength: row.subtree_length,
                radius: row.radius,
                quality: row.quality
            ))
        }
    }
    var message: [String] = []
    if let pointer = raw.messages {
        message.reserveCapacity(Int(raw.message_count))
        for index in 0..<Int(raw.message_count) {
            if let line = pointer[index] {
                message.append(String(cString: line))
            }
        }
    }
    let name = raw.name.flatMap { value -> String? in
        let text = String(cString: value)
        return text.isEmpty ? nil : text
    }
    let crs = raw.crs.map { String(cString: $0) } ?? ""
    return QSM(cylinders: cylinders, id: Int(raw.id), name: name, message: message, crs: crs)
}

func makeQSF(_ raw: ArborBridgeQSF) -> QSF {
    var models: [QSM] = []
    if let pointer = raw.models {
        models.reserveCapacity(Int(raw.count))
        for index in 0..<Int(raw.count) {
            models.append(makeQSM(pointer[index]))
        }
    }
    return QSF(models: models)
}

func makeDBH(_ raw: ArborBridgeDBH) -> DBH {
    DBH(dbh: raw.dbh, x: raw.x, y: raw.y, z: raw.z, nx: raw.nx, ny: raw.ny, nz: raw.nz)
}

private func integers(_ set: Int32, _ pointer: UnsafeMutablePointer<Int32>?, _ count: Int) -> [Int32]? {
    guard set != 0 else { return nil }
    guard count > 0, let pointer else { return [] }
    return Array(UnsafeBufferPointer(start: pointer, count: count))
}

private func doubles(_ set: Int32, _ pointer: UnsafeMutablePointer<Double>?, _ count: Int) -> [Double]? {
    guard set != 0 else { return nil }
    guard count > 0, let pointer else { return [] }
    return Array(UnsafeBufferPointer(start: pointer, count: count))
}

private func floats(_ set: Int32, _ pointer: UnsafeMutablePointer<Float>?, _ count: Int) -> [Float]? {
    guard set != 0 else { return nil }
    guard count > 0, let pointer else { return [] }
    return Array(UnsafeBufferPointer(start: pointer, count: count))
}

private func words(_ pointer: UnsafeMutablePointer<UInt16>?, _ count: Int) -> [UInt16] {
    guard count > 0, let pointer else { return [] }
    return Array(UnsafeBufferPointer(start: pointer, count: count))
}
