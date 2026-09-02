//! Shared lane-id mapping between the `.xosc` and `.xodr` exporters (SW-15).
//!
//! The two artifacts must refer to the same physical lane by the same id, but
//! they use different conventions:
//!
//! - The scenario's internal lane index (`State::cartesian::lane`, 0-based,
//!   derived from `py` — SW-10) is direction-independent:
//!   `py = lane*lane_width + lane_width/2` regardless of whether that lane
//!   carries forward or backward traffic (see `cartesian.rs`).
//! - OpenDRIVE ids are split by direction relative to the road's reference
//!   line: forward lanes (`RoadSpec::lane_directions[i] == 1`) are numbered
//!   with negative ids on the right, backward lanes (`== -1`) with positive
//!   ids on the left, each counted outermost-first within its side.
//!
//! `xodr_exporter::build_lane_section` performs this exact assignment when it
//! builds the `<road><lanes><laneSection>`'s `<right>`/`<left>` lists. This
//! function is the single source of truth for the mapping, so the `.xosc`
//! `LanePosition` elements this crate emits cannot disagree with the `.xodr`
//! lane ids describing the same road. `RoadSpec::validate` guarantees
//! `lane_directions` is a single forward-then-backward block, which is what
//! makes the assignment well-defined (SW-16 E2).
//!
//! `xodr_exporter.rs` is not on this issue's touch list (SW-16 owns it), so
//! it still computes this mapping inline rather than calling this function —
//! see the SW-15 report for why that adoption did not happen here.

use crate::dsl::types::RoadSpec;

/// The OpenDRIVE road id used by every road this crate exports. There is
/// always exactly one `<road>` per scenario (`xodr_exporter::export_to_xodr`
/// pushes a single `Road { id: "0", .. }`), so this is a crate-wide constant
/// rather than something derived per scenario.
pub const XODR_ROAD_ID: &str = "0";

/// Map a scenario's 0-based lane index (as carried on `State::cartesian::lane`)
/// to the OpenDRIVE lane id that identifies the same physical lane in the
/// companion `.xodr` file.
///
/// Mirrors `xodr_exporter::build_lane_section`'s id assignment exactly:
/// forward lanes get ids `-n_forward, ..., -1` in index order; backward lanes
/// get ids `1, 2, ...` in index order.
///
/// An out-of-range `lane` index (which `RoadSpec::validate` plus the solver's
/// derivation of `lane` from `py` should never produce) falls back to the
/// outermost lane on the side implied by `RoadSpec::get_lane_direction`,
/// rather than panicking.
#[must_use]
pub fn lane_index_to_xodr_id(road: &RoadSpec, lane: usize) -> i64 {
    let n_forward: i64 = road
        .lane_directions
        .iter()
        .filter(|&&d| d == 1)
        .count()
        .try_into()
        .unwrap_or(i64::MAX);
    let mut right_id = -n_forward;
    let mut left_id = 1_i64;

    for (i, &direction) in road.lane_directions.iter().enumerate() {
        let assigned = if direction == 1 {
            let id = right_id;
            right_id += 1;
            id
        } else {
            let id = left_id;
            left_id += 1;
            id
        };
        if i == lane {
            return assigned;
        }
    }

    if road.get_lane_direction(lane) == 1 {
        -1
    } else {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn road(directions: Vec<i32>) -> RoadSpec {
        RoadSpec {
            num_lanes: directions.len(),
            lane_width: 3.5,
            lane_directions: directions,
            road_length: None,
        }
    }

    /// `simple_bidirectional`'s road: `[1, 1, -1, -1]` — the acceptance
    /// criterion's own example, ids `-2, -1, +1, +2`.
    #[test]
    fn test_bidirectional_four_lane_ids() {
        let r = road(vec![1, 1, -1, -1]);
        assert_eq!(lane_index_to_xodr_id(&r, 0), -2);
        assert_eq!(lane_index_to_xodr_id(&r, 1), -1);
        assert_eq!(lane_index_to_xodr_id(&r, 2), 1);
        assert_eq!(lane_index_to_xodr_id(&r, 3), 2);
    }

    /// `cut_in_left`'s two-lane, all-forward road.
    #[test]
    fn test_all_forward_two_lane_ids() {
        let r = road(vec![1, 1]);
        assert_eq!(lane_index_to_xodr_id(&r, 0), -2);
        assert_eq!(lane_index_to_xodr_id(&r, 1), -1);
    }

    /// A single all-backward lane sits at id +1, not -1: direction alone
    /// determines which side of the reference line the id lands on.
    #[test]
    fn test_single_backward_lane() {
        let r = road(vec![-1]);
        assert_eq!(lane_index_to_xodr_id(&r, 0), 1);
    }

    /// Three forward lanes then two backward: forward ids count outermost
    /// (-3) to innermost (-1); backward ids count innermost (1) to outermost
    /// (2), matching `xodr_exporter::build_lane_section`.
    #[test]
    fn test_three_forward_two_backward() {
        let r = road(vec![1, 1, 1, -1, -1]);
        assert_eq!(lane_index_to_xodr_id(&r, 0), -3);
        assert_eq!(lane_index_to_xodr_id(&r, 1), -2);
        assert_eq!(lane_index_to_xodr_id(&r, 2), -1);
        assert_eq!(lane_index_to_xodr_id(&r, 3), 1);
        assert_eq!(lane_index_to_xodr_id(&r, 4), 2);
    }
}
