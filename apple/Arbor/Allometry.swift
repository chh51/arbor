// @file Allometry.swift
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

/// One allometric model name and its reference, the columns R returns.
public struct Allometry: Sendable {
    public var model: String
    public var url: String

    public init(model: String, url: String) {
        self.model = model
        self.url = url
    }
}

/// Names and references of the built-in allometric models.
///
/// The R function also draws the curves. This function returns the table only.
public func availableAllometries() -> [Allometry] {
    [
        Allometry(model: "CostaCysneiros2020", url: "https://doi.org/10.1139/cjfr-2020-0060"),
        Allometry(model: "Griese2025", url: "https://doi.org/10.1038/s41597-025-06421-7"),
        Allometry(model: "Chenge2020", url: "https://doi.org/10.1016/j.tfp.2020.100051"),
        Allometry(model: "FSMiombo", url: "Unpublished, by r-lidar")
    ]
}
