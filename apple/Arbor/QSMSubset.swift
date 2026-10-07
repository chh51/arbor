// @file QSMSubset.swift
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

/// Keep the stem cylinders of one model.
///
/// The stem is branch order 1. Attributes `id`, `name`, `message`, and `crs` are kept.
public func qsmStem(_ model: QSM) throws -> QSM {
    try Engine.qsmStem(model)
}

/// Stem of each model. Empty results are kept, matching `qsm_stem` on a `qsf`.
public func qsmStem(_ forest: QSF) throws -> QSF {
    QSF(models: try forest.models.map { try qsmStem($0) }, crs: forest.crs)
}

/// Keep the merchantable cylinders of one model.
///
/// - Parameter merchantableRadius: Metres. Default 0.045 (9 cm diameter), the Canadian standard.
/// - Parameter merchantableLength: Metres. Default 1, the Canadian standard.
public func qsmMerchantable(_ model: QSM, merchantableRadius: Double = 0.045, merchantableLength: Double = 1) throws -> QSM {
    try Engine.qsmMerchantable(model, radius: merchantableRadius, length: merchantableLength)
}

/// Merchantable cylinders of each model. Models with no cylinders left are dropped.
public func qsmMerchantable(_ forest: QSF, merchantableRadius: Double = 0.045, merchantableLength: Double = 1) throws -> QSF {
    var kept: [QSM] = []
    for model in forest.models {
        let subset = try qsmMerchantable(model, merchantableRadius: merchantableRadius, merchantableLength: merchantableLength)
        if !subset.cylinders.isEmpty {
            kept.append(subset)
        }
    }
    return QSF(models: kept, crs: forest.crs)
}

/// Keep whole trees whose DBH is large enough.
///
/// This does not trim cylinders. A tree is kept when `dbh / 2` is greater than
/// `merchantableRadius`. Default radius is 0.045 m.
///
/// ## See Also
/// ``qsmMerchantable(_:merchantableRadius:merchantableLength:)``
public func qsfMerchantable(_ forest: QSF, merchantableRadius: Double = 0.045) throws -> QSF {
    var mask = [Bool]()
    mask.reserveCapacity(forest.count)
    for model in forest.models {
        let measure = try qsmDbh(model)
        mask.append(measure.dbh / 2 > merchantableRadius)
    }
    return forest[mask]
}

/// Set the radius to 0 for cylinders closer to the root than `stumpHeight`.
///
/// The cylinders stay in the model. Default stump height is 0.15 m.
public func qsmNoStump(_ model: QSM, stumpHeight: Double = 0.15) -> QSM {
    for index in model.cylinders.indices where model.cylinders[index].distToRoot < stumpHeight {
        model.cylinders[index].radius = 0
    }
    return model
}

/// ``qsmNoStump(_:stumpHeight:)`` on each model. Models with no cylinders are dropped.
public func qsmNoStump(_ forest: QSF, stumpHeight: Double = 0.15) -> QSF {
    let updated = forest.models.map { qsmNoStump($0, stumpHeight: stumpHeight) }
    return QSF(models: updated.filter { !$0.cylinders.isEmpty }, crs: forest.crs)
}
