// @file QSFIO.swift
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

/// Export format for ``qsfWrite(_:to:format:binary:)``.
public enum QSFFormat: String, Sendable, CaseIterable {
    case qsm
    case qsf
    case ply
    case obj
    case stl
    case csv
    case txt
}

/// Write a forest.
///
/// A path with an extension is one file. That mode allows `qsf`, `obj`, `ply`, and `stl`.
/// `qsm`, `csv`, and `txt` cannot combine several trees into one file. A path with no
/// extension is a directory: each format is written under it, and a `.qsm` export also
/// writes a sibling `.qsf` manifest. `format` is ignored for a single file.
///
/// - Parameter format: Defaults to `qsm` and `obj`.
/// - Parameter binary: ASCII or binary where the format has both. Default true.
public func qsfWrite(_ forest: QSF, to path: String, format: [QSFFormat] = [.qsm, .obj], binary: Bool = true) throws {
    let ext = URL(fileURLWithPath: path).pathExtension.lowercased()
    if !ext.isEmpty {
        guard let single = QSFFormat(rawValue: ext) else {
            throw ArborError.invalidArgument("Unsupported file extension '.\(ext)'. Must be one of: qsm, qsf, ply, obj, stl, csv, txt")
        }
        let singleFile: Set<QSFFormat> = [.qsf, .obj, .ply, .stl]
        guard singleFile.contains(single) else {
            let stem = URL(fileURLWithPath: path).deletingPathExtension().lastPathComponent
            throw ArborError.invalidArgument("'.\(ext)' does not support combining several QSMs into a single file. Single-file output is only supported for: qsf, obj, ply, stl. Use directory mode instead, e.g. qsfWrite(forest, to: \"\(stem)\", format: [.\(ext)]).")
        }
        try Engine.writeQSF(forest, to: path, format: ext, binary: binary)
        return
    }
    guard !format.isEmpty else {
        throw ArborError.invalidArgument("'format' must be a non-empty character vector.")
    }
    for item in format {
        try Engine.writeQSF(forest, to: path, format: item.rawValue, binary: binary)
    }
}

/// Read a `.qsf` manifest and the `.qsm` files it names.
///
/// The manifest is an index of relative paths. Models come back ordered by tree id,
/// matching `qsf_read`.
///
/// ## See Also
/// ``qsfWrite(_:to:format:binary:)``
public func qsfRead(_ path: String) throws -> QSF {
    guard FileManager.default.fileExists(atPath: path) else {
        throw ArborError.invalidArgument("File not found: \(path)")
    }
    let forest = try Engine.readQSF(path)
    forest.models.sort { ($0.id ?? .min) < ($1.id ?? .min) }
    return forest
}
