// @file ArborError.swift
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

/// Failure from a processing call.
///
/// These are the Swift form of the R package's `stop()` conditions, plus the
/// gap where a function still has to enter the C++ engine.
public enum ArborError: Error, Equatable, Sendable {
    /// A column the R function requires is absent.
    case missingAttribute(String)
    /// An argument fails a check the R function makes before it runs.
    case invalidArgument(String)
    /// This function's work is in the C++ engine, and the Swift target does not call that engine yet.
    case engineNotConnected(String)
}
