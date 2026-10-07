// @file QSMIO.swift
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

import Foundation

/// Write one model.
///
/// `.csv` and `.txt` are written here, sorted by `cyl_ID`, with the C++ header
/// `startX,startY,startZ,endX,endY,endZ,cyl_ID,parent_ID,axis_ID,branch_order,dist_to_root,subtree_length,radius,quality`.
/// `.qsm`, `.ply`, `.obj`, `.stl`, and `.qbf` are written by the C++ engine.
///
/// - Parameter binary: Used by formats that have an ASCII and a binary form.
public func qsmWrite(_ model: QSM, to file: String, binary: Bool = true) throws {
    let ext = URL(fileURLWithPath: file).pathExtension.lowercased()
    switch ext {
    case "csv", "txt":
        try writeQSMTable(model, to: file)
    case "qsm", "ply", "obj", "stl", "qbf":
        try Engine.writeQSM(model, to: file, binary: binary)
    default:
        throw ArborError.invalidArgument("Unsupported file extension '.\(ext)'. Must be one of: qsm, ply, obj, stl, csv, txt")
    }
}

/// Read one model from `.csv` or from a native `.qsm` file.
///
/// CSV columns `branchOrder`, `parent_segment_ID`, and `segment_ID` are renamed to
/// `branch_order`, `parent_ID`, and `cyl_ID`. The file name, without its extension,
/// is stored in ``QSM/name``.
public func qsmRead(_ path: String) throws -> QSM {
    guard FileManager.default.fileExists(atPath: path) else {
        throw ArborError.invalidArgument("File not found: \(path)")
    }
    let ext = URL(fileURLWithPath: path).pathExtension.lowercased()
    if ext == "csv" || ext == "txt" {
        return try readQSMTable(path)
    }
    return try Engine.readQSM(path)
}

private let qsmHeader = "startX,startY,startZ,endX,endY,endZ,cyl_ID,parent_ID,axis_ID,branch_order,dist_to_root,subtree_length,radius,quality"

private func writeQSMTable(_ model: QSM, to file: String) throws {
    var lines = [qsmHeader]
    let ordered = model.cylinders.sorted { $0.cylID < $1.cylID }
    for cylinder in ordered {
        lines.append([
            format3(cylinder.startX),
            format3(cylinder.startY),
            format3(cylinder.startZ),
            format3(cylinder.endX),
            format3(cylinder.endY),
            format3(cylinder.endZ),
            String(cylinder.cylID),
            String(cylinder.parentID),
            String(cylinder.axisID),
            String(cylinder.branchOrder),
            format3(cylinder.distToRoot),
            format3(cylinder.subtreeLength),
            format3(cylinder.radius),
            String(cylinder.quality)
        ].joined(separator: ","))
    }
    do {
        try lines.joined(separator: "\n").appending("\n").write(toFile: file, atomically: true, encoding: .utf8)
    } catch {
        throw ArborError.invalidArgument("Cannot open CSV file: \(file)")
    }
}

private func readQSMTable(_ path: String) throws -> QSM {
    let text = try String(contentsOfFile: path, encoding: .utf8)
    let rows = text.split(whereSeparator: \.isNewline).map { String($0) }.filter { !$0.isEmpty }
    guard let headerLine = rows.first else {
        throw ArborError.invalidArgument("QSM table is empty: \(path)")
    }
    let header = headerLine.split(separator: ",", omittingEmptySubsequences: false).map { String($0).trimmingCharacters(in: .whitespaces) }
    let renamed = header.map { name -> String in
        switch name {
        case "branchOrder": return "branch_order"
        case "parent_segment_ID": return "parent_ID"
        case "segment_ID": return "cyl_ID"
        default: return name
        }
    }
    func column(_ name: String) throws -> Int {
        guard let index = renamed.firstIndex(of: name) else {
            throw ArborError.invalidArgument("Missing required QSM column: \(name)")
        }
        return index
    }
    let startX = try column("startX")
    let startY = try column("startY")
    let startZ = try column("startZ")
    let endX = try column("endX")
    let endY = try column("endY")
    let endZ = try column("endZ")
    let cylID = try column("cyl_ID")
    let parentID = try column("parent_ID")
    let axisID = try column("axis_ID")
    let branchOrder = try column("branch_order")
    let distToRoot = try column("dist_to_root")
    let subtreeLength = try column("subtree_length")
    let radius = try column("radius")
    let quality = renamed.firstIndex(of: "quality")
    let width = renamed.count

    var cylinders: [Cylinder] = []
    for line in rows.dropFirst() {
        let fields = line.split(separator: ",", omittingEmptySubsequences: false).map { String($0).trimmingCharacters(in: .whitespaces) }
        guard fields.count >= width else { continue }
        func number(_ index: Int) throws -> Double {
            guard let value = Double(fields[index]) else {
                throw ArborError.invalidArgument("Invalid number in \(path)")
            }
            return value
        }
        func integer(_ index: Int) throws -> Int32 {
            guard let value = Int32(fields[index].split(separator: ".").first.map(String.init) ?? fields[index]) else {
                throw ArborError.invalidArgument("Invalid integer in \(path)")
            }
            return value
        }
        cylinders.append(Cylinder(
            startX: try number(startX),
            startY: try number(startY),
            startZ: try number(startZ),
            endX: try number(endX),
            endY: try number(endY),
            endZ: try number(endZ),
            cylID: try integer(cylID),
            parentID: try integer(parentID),
            axisID: try integer(axisID),
            branchOrder: try integer(branchOrder),
            distToRoot: try number(distToRoot),
            subtreeLength: try number(subtreeLength),
            radius: try number(radius),
            quality: quality.map { (try? integer($0)) ?? 0 } ?? 0
        ))
    }
    let name = URL(fileURLWithPath: path).deletingPathExtension().lastPathComponent
    return QSM(cylinders: cylinders, name: name)
}

private func format3(_ value: Double) -> String {
    String(format: "%.3f", value)
}
