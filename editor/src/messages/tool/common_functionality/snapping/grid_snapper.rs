use super::*;

use crate::messages::portfolio::document::utility_types::misc::{GridSnapTarget, GridSnapping, GridType, SnapTarget};

use glam::DVec2;
use graphene_core::renderer::Quad;

struct Line {
	pub point: DVec2,
	pub direction: DVec2,
}

#[derive(Clone, Debug, Default)]

pub struct GridSnapper;

impl GridSnapper {
	// Rectangular grid has 4 lines around a point: 2 along the X axis and 2 along the Y axis.
	fn get_snap_lines_rectangular(&self, document_point: DVec2, snap_data: &SnapData, spacing: DVec2) -> Vec<Line> {
		let Some(spacing) = GridSnapping::compute_rectangle_spacing(spacing, &snap_data.document.document_ptz) else {
			return Vec::new();
		};

		let origin = snap_data.document.snapping_state.grid.origin;

		let delta = document_point - origin;
		let factors = (delta / spacing).to_array();

		let mut lines = Vec::with_capacity(4);

		for (i, (direction, perpendicular)) in [DVec2::X, DVec2::Y].iter().zip(&[DVec2::Y, DVec2::X]).enumerate() {
			let direction = *direction;
			let perpendicular = *perpendicular;

			let floor_factor = factors[i].floor();
			let ceil_factor = factors[i].ceil();

			let floor_point = perpendicular * (floor_factor * spacing[i] + origin[i]);
			let ceil_point = perpendicular * (ceil_factor * spacing[i] + origin[i]);

			lines.push(Line { direction, point: floor_point });
			lines.push(Line { direction, point: ceil_point });
		}

		lines
	}
	// Isometric grid has 6 lines around a point: 2 along Y-axis, 2 at angle_a, and 2 at angle_b.
	fn get_snap_lines_isometric(&self, document_point: DVec2, snap_data: &SnapData, y_axis_spacing: f64, angle_a: f64, angle_b: f64) -> Vec<Line> {
		let tan_a = angle_a.to_radians().tan();
		let tan_b = angle_b.to_radians().tan();

		let spacing_x = y_axis_spacing / (tan_a + tan_b);
		let spacing = DVec2::new(spacing_x, y_axis_spacing);

		let Some(spacing_multiplier) = GridSnapping::compute_isometric_multiplier(y_axis_spacing, tan_a + tan_b, &snap_data.document.document_ptz) else {
			return Vec::new();
		};

		let spacing = spacing * spacing_multiplier;
		let origin = snap_data.document.snapping_state.grid.origin;
		let delta = document_point - origin;

		let mut lines = Vec::with_capacity(6);

		// Lines along Y-axis
		let factor_x = delta.x / spacing.x;
		let x_floor = factor_x.floor() * spacing.x + origin.x;
		let x_ceil = factor_x.ceil() * spacing.x + origin.x;

		lines.push(Line {
			point: DVec2::new(x_floor, 0.0),
			direction: DVec2::Y,
		});
		lines.push(Line {
			point: DVec2::new(x_ceil, 0.0),
			direction: DVec2::Y,
		});

		// Lines at angle_a
		let dir_a = DVec2::new(1.0, -tan_a);
		let y_projected_a = delta.y + tan_a * delta.x + origin.y;
		let factor_a = (y_projected_a - origin.y) / spacing.y;
		let y_a_floor = factor_a.floor() * spacing.y + origin.y;
		let y_a_ceil = factor_a.ceil() * spacing.y + origin.y;

		lines.push(Line {
			point: DVec2::new(origin.x, y_a_floor),
			direction: dir_a,
		});
		lines.push(Line {
			point: DVec2::new(origin.x, y_a_ceil),
			direction: dir_a,
		});

		// Lines at angle_b
		let dir_b = DVec2::new(1.0, tan_b);
		let y_projected_b = delta.y - tan_b * delta.x + origin.y;
		let factor_b = (y_projected_b - origin.y) / spacing.y;
		let y_b_floor = factor_b.floor() * spacing.y + origin.y;
		let y_b_ceil = factor_b.ceil() * spacing.y + origin.y;

		lines.push(Line {
			point: DVec2::new(origin.x, y_b_floor),
			direction: dir_b,
		});
		lines.push(Line {
			point: DVec2::new(origin.x, y_b_ceil),
			direction: dir_b,
		});

		lines
	}
	fn get_snap_lines(&self, document_point: DVec2, snap_data: &mut SnapData) -> Vec<Line> {
		match snap_data.document.snapping_state.grid.grid_type {
			GridType::Rectangle { spacing } => self.get_snap_lines_rectangular(document_point, snap_data, spacing),
			GridType::Isometric { y_axis_spacing, angle_a, angle_b } => self.get_snap_lines_isometric(document_point, snap_data, y_axis_spacing, angle_a, angle_b),
		}
	}

	pub fn free_snap(&mut self, snap_data: &mut SnapData, point: &SnapCandidatePoint, snap_results: &mut SnapResults) {
		let tolerance = snap_tolerance(snap_data.document);
		let tolerance_squared = tolerance * tolerance; // For squared distance comparisons

		let snapping_state = &snap_data.document.snapping_state;
		let grid_line_enabled = snapping_state.target_enabled(SnapTarget::Grid(GridSnapTarget::Line)) || snapping_state.target_enabled(SnapTarget::Grid(GridSnapTarget::Intersection));
		let normal_target = SnapTarget::Grid(GridSnapTarget::LineNormal);
		let normal_enabled = snapping_state.target_enabled(normal_target);

		let lines = self.get_snap_lines(point.document_point, snap_data);

		for line in lines {
			let vector_to_point = point.document_point - line.point;
			let projected_vector = vector_to_point.project_onto(line.direction);
			let projected = line.point + projected_vector;

			let distance_squared = point.document_point.distance_squared(projected);
			if !distance_squared.is_finite() || distance_squared > tolerance_squared {
				continue;
			}

			if grid_line_enabled {
				snap_results.grid_lines.push(SnappedLine {
					direction: line.direction,
					point: SnappedPoint {
						snapped_point_document: projected,
						source: point.source,
						target: SnapTarget::Grid(GridSnapTarget::Line),
						source_bounds: point.quad,
						distance: distance_squared.sqrt(),
						tolerance,
						..Default::default()
					},
				});
			}

			if normal_enabled {
				for &neighbor in &point.neighbors {
					let neighbor_vector = neighbor - line.point;
					let neighbor_projected_vector = neighbor_vector.project_onto(line.direction);
					let neighbor_projected = line.point + neighbor_projected_vector;

					let neighbor_distance_squared = point.document_point.distance_squared(neighbor_projected);

					if !neighbor_distance_squared.is_finite() || neighbor_distance_squared > tolerance_squared {
						continue;
					}
					snap_results.points.push(SnappedPoint {
						snapped_point_document: neighbor_projected,
						source: point.source,
						source_bounds: point.quad,
						target: normal_target,
						distance: neighbor_distance_squared.sqrt(),
						tolerance,
						..Default::default()
					});
				}
			}
		}
	}

	pub fn constrained_snap(&mut self, snap_data: &mut SnapData, point: &SnapCandidatePoint, snap_results: &mut SnapResults, constraint: SnapConstraint) {
		let tolerance = snap_tolerance(snap_data.document);
		let tolerance_squared = tolerance * tolerance;
		let snapping_enabled = snap_data.document.snapping_state.target_enabled(SnapTarget::Grid(GridSnapTarget::Line));

		if !snapping_enabled {
			return;
		}

		let projected = constraint.projection(point.document_point);

		let lines = self.get_snap_lines(projected, snap_data);

		let (constraint_start, constraint_direction) = match constraint {
			SnapConstraint::Line { origin, direction } => {
				let dir = direction.normalize_or_zero();
				// Skip if direction is zero vector
				if dir.length_squared() == 0.0 {
					return;
				}
				(origin, dir)
			}
			SnapConstraint::Direction(direction) => {
				let dir = direction.normalize_or_zero();
				// Skip if direction is zero vector
				if dir.length_squared() == 0.0 {
					return;
				}
				(projected, dir)
			}
			_ => unimplemented!(),
		};

		for line in lines {
			if line.direction.length_squared() == 0.0 {
				continue;
			}

			let Some(intersection) = Quad::intersect_rays(line.point, line.direction, constraint_start, constraint_direction) else {
				continue;
			};

			let distance_squared = intersection.distance_squared(point.document_point);

			if !distance_squared.is_finite() || distance_squared > tolerance_squared {
				continue;
			}

			let distance = distance_squared.sqrt();

			snap_results.points.push(SnappedPoint {
				snapped_point_document: intersection,
				source: point.source,
				target: SnapTarget::Grid(GridSnapTarget::Line),
				at_intersection: false,
				constrained: true,
				source_bounds: point.quad,
				distance,
				tolerance,
				..Default::default()
			});
		}
	}
}
