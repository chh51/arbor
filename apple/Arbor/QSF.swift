// @file QSF.swift
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

/// A quantitative structure forest: an ordered collection of ``QSM`` values.
///
/// This is the Swift form of an R `qsf` list. ``subscript(mask:)`` is `[.qsf`.
/// Trees flagged with ``flagBuffer(_:seeds:buffer:)`` or ``flagSmallTrees(_:maxHeight:)``
/// are left out of ``qsf(_:minHeight:params:)``.
public final class QSF {
    public var models: [QSM]
    private var storedCRS: String

    /// CRS of the first model, or the CRS last assigned when the forest is empty.
    ///
    /// Assigning writes the same string onto every model. Swift form of `st_crs<-`.
    public var crs: String {
        get { models.first?.crs ?? storedCRS }
        set {
            storedCRS = newValue
            for model in models {
                model.crs = newValue
            }
        }
    }

    public var count: Int { models.count }

    public init(models: [QSM] = [], crs: String = "") {
        self.models = models
        self.storedCRS = crs
        if !crs.isEmpty {
            for model in models where model.crs.isEmpty {
                model.crs = crs
            }
        }
    }

    public subscript(position: Int) -> QSM {
        get { models[position] }
        set { models[position] = newValue }
    }

    /// Subset, preserving the forest. Swift form of `[.qsf`.
    public subscript(bounds: Range<Int>) -> QSF {
        QSF(models: Array(models[bounds]), crs: crs)
    }

    /// Logical index. Swift form of `[.qsf` with a logical vector.
    public subscript(mask: [Bool]) -> QSF {
        precondition(mask.count == models.count, "mask length must match the forest")
        let kept = zip(models, mask).compactMap { model, include in include ? model : nil }
        return QSF(models: kept, crs: crs)
    }
}

/// Build a QSM for each tree taller than `minHeight`.
///
/// Flagged buffer and understory points are excluded. The default height matches
/// ``flagSmallTrees(_:maxHeight:)`` so a tree under 2 m is not modelled even when
/// that flag was not set. See the
/// [Arbor book](https://r-lidar.github.io/arbor_book/).
///
/// - Parameter cloud: A cloud with semantic and instance segmentation.
/// - Parameter minHeight: Metres. Default 2.
/// - Parameter params: See ``arborParametersDefault``.
/// - Returns: Models ordered by tree id. `crs` is copied from the cloud.
///
/// ## See Also
/// ``qsm(_:params:)``
/// ``qsfLog(_:)``
public func qsf(_ cloud: PointCloud, minHeight: Double = 2, params: ArborParameters = arborParametersDefault) throws -> QSF {
    try cloud.validate()
    let forest = try Engine.qsf(cloud, minHeight: minHeight, params: params)
    forest.models.sort { ($0.id ?? .min) < ($1.id ?? .min) }
    forest.crs = cloud.crs
    return forest
}
