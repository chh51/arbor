// @file QSMDBH.swift
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

/// Diameter at breast height, and the circle that produced it.
///
/// `dbh`, `x`, `y`, `z`, `nx`, `ny`, and `nz` match the R data frame.
/// `treeID` is set when the value comes from a forest.
public struct DBH: Sendable {
    public var dbh: Double
    public var x: Double
    public var y: Double
    public var z: Double
    public var nx: Double
    public var ny: Double
    public var nz: Double
    public var treeID: Int?

    public init(
        dbh: Double,
        x: Double,
        y: Double,
        z: Double,
        nx: Double,
        ny: Double,
        nz: Double,
        treeID: Int? = nil
    ) {
        self.dbh = dbh
        self.x = x
        self.y = y
        self.z = z
        self.nx = nx
        self.ny = ny
        self.nz = nz
        self.treeID = treeID
    }
}

/// Diameter at breast height for one model.
///
/// - Parameter breastHeight: Metres above the model origin. Default 1.30.
/// - Returns: Diameter in metres, the circle centre, and its normal.
public func qsmDbh(_ model: QSM, breastHeight: Double = 1.30) throws -> DBH {
    try Engine.qsmDbh(model, breastHeight: breastHeight)
}

/// Diameter at breast height for each model.
///
/// `treeID` on each result is the model's `id`, matching the `treeID` column
/// R adds when `qsm_dbh` is called on a `qsf`.
public func qsmDbh(_ forest: QSF, breastHeight: Double = 1.30) throws -> [DBH] {
    try forest.models.map { model in
        var measure = try qsmDbh(model, breastHeight: breastHeight)
        measure.treeID = model.id
        return measure
    }
}
