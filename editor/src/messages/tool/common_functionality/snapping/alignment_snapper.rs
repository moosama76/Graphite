use super::*;
use crate::messages::portfolio::document::utility_types::misc::*;

use graphene_core::renderer::Quad;

use glam::{DAffine2, DVec2};

#[derive(Clone, Debug, Default)]
pub struct AlignmentSnapper {
	bounding_box_points: Vec<SnapCandidatePoint>,
}

impl AlignmentSnapper {
	pub fn collect_bounding_box_points(&mut self, snap_data: &mut SnapData, first_point: bool) {
		if !first_point {
			return;
		}

		let document = snap_data.document;

		self.bounding_box_points.clear();
		if !document.snapping_state.bounding_box.align_with_edges {
			return;
		}

		for layer in document.metadata().all_layers() {
			if !document.network_interface.is_artboard(&layer.to_node(), &[]) || snap_data.ignore.contains(&layer) {
				continue;
			}

			if document.snapping_state.target_enabled(SnapTarget::Artboard(ArtboardSnapTarget::CornerPoint)) {
				let Some(bounds) = document.metadata().bounding_box_with_transform(layer, document.metadata().transform_to_document(layer)) else {
					continue;
				};

				get_bbox_points(Quad::from_box(bounds), &mut self.bounding_box_points, BBoxSnapValues::ALIGN_ARTBOARD, document);
			}
		}
		for &layer in snap_data.alignment_candidates.map_or([].as_slice(), |candidates| candidates.as_slice()) {
			if snap_data.ignore_bounds(layer) {
				continue;
			}
			let Some(bounds) = document.metadata().bounding_box_with_transform(layer, DAffine2::IDENTITY) else {
				continue;
			};

			let quad = document.metadata().transform_to_document(layer) * Quad::from_box(bounds);
			let values = BBoxSnapValues::ALIGN_BOUNDING_BOX;
			get_bbox_points(quad, &mut self.bounding_box_points, values, document);
		}
	}

	pub fn snap_bbox_points(&mut self, snap_data: &mut SnapData, point: &SnapCandidatePoint, snap_results: &mut SnapResults, constraint: SnapConstraint, config: SnapTypeConfiguration) {
		// Collect bounding box points if needed
		if !config.use_existing_candidates {
			self.collect_bounding_box_points(snap_data, true);
		}

		// Get unselected geometry if snapping target is enabled
		let unselected_geometry = if snap_data.document.snapping_state.target_enabled(SnapTarget::Alignment(AlignmentSnapTarget::AlignWithAnchorPoint)) {
			snap_data.node_snap_cache.as_ref().map(|cache| cache.unselected.as_slice()).unwrap_or(&[])
		} else {
			&[]
		};

		// TODO: snap handle points
		let tolerance = snap_tolerance(snap_data.document);
		let tolerance_squared = tolerance * tolerance;

		let mut snap_x: Option<SnappedPoint> = None;
		let mut snap_y: Option<SnappedPoint> = None;

		for target_point in self.bounding_box_points.iter().chain(unselected_geometry) {
			let target_position = target_point.document_point;

			let (point_on_x, point_on_y) = match constraint {
				SnapConstraint::Line { origin, direction } => (
					Quad::intersect_rays(target_position, DVec2::Y, origin, direction),
					Quad::intersect_rays(target_position, DVec2::X, origin, direction),
				),
				_ => {
					let Some(quad) = target_point.quad.map(|quad| quad.0) else { continue };
					let edges = [quad[1] - quad[0], quad[3] - quad[0]];

					let projections: [Option<DVec2>; 2] = edges.map(|edge| edge.try_normalize().map(|edge_norm| (point.document_point - target_position).project_onto(edge_norm) + target_position));

					(projections[0], projections[1])
				}
			};

			// Update snapping points along X and Y axes
			for (axis, point_on_axis, snap_point) in [("x", point_on_x, &mut snap_x), ("y", point_on_y, &mut snap_y)] {
				let Some(point_on_axis) = point_on_axis else { continue };

				let distance_squared = point.document_point.distance_squared(point_on_axis);
				if distance_squared >= tolerance_squared {
					continue;
				}

				let distance_to_align_target_squared = point_on_axis.distance_squared(target_position);

				let is_better = snap_point
					.as_ref()
					.map_or(true, |existing_snap| distance_to_align_target_squared < existing_snap.distance_to_align_target.powi(2));

				if is_better {
					let distance = distance_squared.sqrt();
					let distance_to_align_target = distance_to_align_target_squared.sqrt();

					let target = if matches!(target_point.target, SnapTarget::Path(_)) {
						SnapTarget::Alignment(AlignmentSnapTarget::AlignWithAnchorPoint)
					} else {
						target_point.target
					};

					let mut new_snap_point = SnappedPoint {
						snapped_point_document: point_on_axis,
						source: point.source, // TODO: map source
						target,
						target_bounds: target_point.quad,
						distance,
						tolerance,
						distance_to_align_target,
						fully_constrained: true,
						at_intersection: matches!(constraint, SnapConstraint::Line { .. }),
						..Default::default()
					};

					// Set alignment target for the corresponding axis
					match axis {
						"x" => new_snap_point.alignment_target_x = Some(target_position),
						"y" => new_snap_point.alignment_target_y = Some(target_position),
						_ => {}
					}

					*snap_point = Some(new_snap_point);
				}
			}
		}

		// Determine the final snapped point
		match (snap_x, snap_y) {
			(Some(snap_x), Some(snap_y)) if !matches!(constraint, SnapConstraint::Line { .. }) => {
				let intersection = DVec2::new(snap_y.snapped_point_document.x, snap_x.snapped_point_document.y);
				let distance_squared = point.document_point.distance_squared(intersection);

				if distance_squared >= tolerance_squared {
					let closer_snap = if snap_x.distance < snap_y.distance { snap_x } else { snap_y };
					snap_results.points.push(closer_snap);
					return;
				}

				let distance = distance_squared.sqrt();

				snap_results.points.push(SnappedPoint {
					snapped_point_document: intersection,
					source: point.source, // TODO: map source
					target: SnapTarget::Alignment(AlignmentSnapTarget::IntersectionPoint),
					target_bounds: snap_x.target_bounds,
					distance,
					tolerance,
					alignment_target_x: snap_x.alignment_target_x,
					alignment_target_y: snap_y.alignment_target_y,
					constrained: true,
					at_intersection: true,
					..Default::default()
				});
			}
			(Some(snap_x), Some(snap_y)) => {
				let closer_snap = if snap_x.distance < snap_y.distance { snap_x } else { snap_y };
				snap_results.points.push(closer_snap);
			}
			(Some(snap_x), _) => snap_results.points.push(snap_x),
			(_, Some(snap_y)) => snap_results.points.push(snap_y),
			_ => {}
		}
	}

	pub fn free_snap(&mut self, snap_data: &mut SnapData, point: &SnapCandidatePoint, snap_results: &mut SnapResults, config: SnapTypeConfiguration) {
		let is_bbox = matches!(point.source, SnapSource::BoundingBox(_));
		let is_path = matches!(point.source, SnapSource::Path(_));
		let path_selected = snap_data.has_manipulators();

		if is_bbox || (is_path && path_selected) || (is_path && point.alignment) {
			self.snap_bbox_points(snap_data, point, snap_results, SnapConstraint::None, config);
		}
	}

	pub fn constrained_snap(&mut self, snap_data: &mut SnapData, point: &SnapCandidatePoint, snap_results: &mut SnapResults, constraint: SnapConstraint, config: SnapTypeConfiguration) {
		let is_bbox = matches!(point.source, SnapSource::BoundingBox(_));
		let is_path = matches!(point.source, SnapSource::Path(_));
		let path_selected = snap_data.has_manipulators();

		if is_bbox || (is_path && path_selected) || (is_path && point.alignment) {
			self.snap_bbox_points(snap_data, point, snap_results, constraint, config);
		}
	}
}
