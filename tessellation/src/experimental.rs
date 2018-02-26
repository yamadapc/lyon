use std::mem;
use std::cmp::Ordering;

use {FillOptions, FillRule, Side};
use geom::math::*;
use geom::{LineSegment, QuadraticBezierSegment, CubicBezierSegment, Arc};
use geom::cubic_to_quadratic::cubic_to_monotonic_quadratics;
use geometry_builder::{GeometryBuilder, VertexId};
use std::ops::Range;
use path_fill::MonotoneTessellator;
use path::builder::*;
use std::{u16, f32};

pub type Vertex = Point;

macro_rules! impl_id {
    ($Name:ident) => (
        #[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
        pub struct $Name(pub u16);

        impl $Name {
            pub const INVALID: Self = $Name(u16::MAX);
            pub fn is_valid(self) -> bool { self != Self::INVALID }
            pub fn to_usize(self) -> usize { self.0 as usize }
            pub fn from_usize(idx: usize) -> Self { $Name(idx as u16) }
        }
    )
}

impl_id!(CtrlPointId);
impl_id!(EndpointId);
impl_id!(SegmentId);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct Segment {
    from: EndpointId,
    to: EndpointId,
    ctrl: CtrlPointId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct SubPathInfo {
    range: Range<usize>,
    is_closed: bool,
}

#[derive(Clone, Debug)]
pub struct Path {
    endpoints: Vec<Point>,
    ctrl_points: Vec<Point>,
    segments: Vec<Segment>,
    sub_paths: Vec<SubPathInfo>,
}

impl Path {
    pub fn new() -> Self {
        Path {
            endpoints: Vec::new(),
            ctrl_points: Vec::new(),
            segments: Vec::new(),
            sub_paths: Vec::new(),
        }
    }

    pub fn builder() -> Builder {
        Builder::new()
    }

    fn previous_segment(&self, id: SegmentId) -> SegmentId {
        let idx = id.0 as usize;
        for sp in &self.sub_paths {
            if sp.range.start > idx || sp.range.end <= idx {
                continue;
            }

            return SegmentId::from_usize(
                if idx == sp.range.start { sp.range.end - 1 } else { idx - 1 }
            );
        }

        SegmentId::INVALID
    }

    fn next_segment(&self, id: SegmentId) -> SegmentId {
        let idx = id.0 as usize;
        for sp in &self.sub_paths {
            if sp.range.start > idx || sp.range.end <= idx {
                continue;
            }

            return SegmentId::from_usize(
                if idx == sp.range.end - 1 { sp.range.start } else { idx + 1 }
            );
        }

        SegmentId::INVALID
    }

    fn segment_from(&self, id: SegmentId) -> EndpointId {
        self.segments[id.to_usize()].from
    }

    fn segment_ctrl(&self, id: SegmentId) -> CtrlPointId {
        self.segments[id.to_usize()].ctrl
    }

    fn segment_to(&self, id: SegmentId) -> EndpointId {
        let id = self.next_segment(id);
        self.segments[id.to_usize()].from
    }

    fn endpoint(&self, id: EndpointId) -> Point {
        if id.is_valid() {
            return self.endpoints[id.0 as usize];
        }

        point(f32::NAN, f32::NAN)
    }

    fn ctrl_point(&self, id: CtrlPointId) -> Point {
        if id.is_valid() {
            return self.ctrl_points[id.0 as usize];
        }

        point(f32::NAN, f32::NAN)
    }


    fn sort(&self, events: &mut Vec<Event>) {
        for sub_path in &self.sub_paths {
            if sub_path.range.end - sub_path.range.start < 2 {
                continue;
            }
            for i in sub_path.range.clone() {
                events.push(Event {
                    vertex: self.segments[i].from,
                    segment: SegmentId::from_usize(i),
                });
            }
        }

        events.sort_by(|a, b| {
            compare_positions(
                self.endpoints[b.vertex.0 as usize],
                self.endpoints[a.vertex.0 as usize],
            )
        });
    }
}

struct Event {
    vertex: EndpointId,
    segment: SegmentId,
}

pub struct Builder {
    path: Path,
}

impl Builder {
    pub fn new() -> Self {
        Builder {
            path: Path::new(),
        }
    }

    pub fn line_to(&mut self, to_pos: Point) {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        let from = EndpointId((self.path.endpoints.len() - 1) as u16);
        let ctrl = CtrlPointId::INVALID;
        let to = EndpointId(from.0 + 1);

        self.path.endpoints.push(to_pos);

        self.path.segments.push(Segment { from, ctrl, to });
    }

    pub fn move_to(&mut self, to_pos: Point) {
        self.end_sub_path(false);
        self.path.endpoints.push(to_pos);
    }

    pub fn close(&mut self) {
        self.end_sub_path(true);
    }

    pub fn quadratic_bezier_to(&mut self, ctrl_pos: Point, to_pos: Point) {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        // TODO: decompose into monotonic segments.
        QuadraticBezierSegment {
            from: *self.path.endpoints.last().unwrap(),
            ctrl: ctrl_pos,
            to: to_pos,
        }.for_each_monotonic(&mut |monotonic| {
            self.quadratic_bezier_to(
                monotonic.segment().from,
                monotonic.segment().to
            );
        });
    }

    fn monotonic_quadratic_bezier_to(&mut self, ctrl_pos: Point, to_pos: Point) {

        let from = EndpointId((self.path.endpoints.len() - 1) as u16);
        let to = EndpointId(from.0 + 1);
        self.path.endpoints.push(to_pos);

        let ctrl = CtrlPointId((self.path.ctrl_points.len() - 1) as u16);
        self.path.ctrl_points.push(ctrl_pos);

        self.path.segments.push(Segment{ from, ctrl, to });
    }

    pub fn cubic_bezier_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        let tolerance = 0.1;

        cubic_to_monotonic_quadratics(
            &CubicBezierSegment {
                from: *self.path.endpoints.last().unwrap(),
                ctrl1,
                ctrl2,
                to,
            },
            tolerance,
            &mut |monotonic| {
                self.monotonic_quadratic_bezier_to(
                    monotonic.segment().ctrl,
                    monotonic.segment().to,
                );
            }
        );
    }

    pub fn arc(&mut self, center: Point, radii: Vector, sweep_angle: Angle, x_rotation: Angle) {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        let from = *self.path.endpoints.last().unwrap();
        let start_angle = (from - center).angle_from_x_axis() - x_rotation;

        Arc {
            center,
            radii,
            start_angle,
            sweep_angle,
            x_rotation,
        }.for_each_quadratic_bezier(
            &mut |curve| { self.quadratic_bezier_to(curve.from, curve.to); }
        );
    }

    fn end_sub_path(&mut self, is_closed: bool) {
        let mut sp_end = self.path.segments.len();
        let sp_start = self.path.sub_paths.last()
            .map(|sp| sp.range.end)
            .unwrap_or(0);

        if sp_end >= sp_start {
            if is_closed && !self.path.endpoints.is_empty() {
                let first = self.path.sub_paths.last().map(|sp|{
                    self.path.segments[sp.range.start].from.0 as usize
                }).unwrap_or(0);
                let first_point = self.path.endpoints[first];
                self.line_to(first_point);

                sp_end += 1;
            }

            self.path.sub_paths.push(SubPathInfo {
                range: sp_start..sp_end,
                is_closed,
            });
        }
    }

    pub fn build(self) -> Path {
        self.path
    }
}

impl FlatPathBuilder for Builder {
    type PathType = Path;

    fn move_to(&mut self, to: Point) { self.move_to(to); }

    fn line_to(&mut self, to: Point) { self.line_to(to); }

    fn close(&mut self) { self.close(); }

    fn build(self) -> Path { self.build() }

    fn build_and_reset(&mut self) -> Path {
        let mut tmp = Builder::new();
        mem::swap(self, &mut tmp);
        tmp.build()
    }

    fn current_position(&self) -> Point {
        let default = Point::new(0.0, 0.0);
        *self.path.endpoints.last().unwrap_or(&default)
    }
}

impl PathBuilder for Builder {
    fn quadratic_bezier_to(&mut self, ctrl: Point, to: Point) {
        self.quadratic_bezier_to(ctrl, to);
    }

    fn cubic_bezier_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.cubic_bezier_to(ctrl1, ctrl2, to);
    }

    fn arc(&mut self, center: Point, radii: Vector, sweep_angle: Angle, x_rotation: Angle) {
        self.arc(center, radii, sweep_angle, x_rotation);
    }
}

pub struct FillTessellator {
    active: ActiveEdges,
    pending_edges: Vec<PendingEdge>,
    fill_rule: FillRule,
    events: Vec<Event>,
    fill: Spans,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Transition {
    In,
    Out,
    None,
}

#[derive(Copy, Clone, Debug)]
struct WindingState {
    span_index: SpanIdx,
    number: i16,
    transition: Transition,
}

impl FillRule {
    fn is_in(&self, winding_number: i16) -> bool {
        match *self {
            FillRule::EvenOdd => { winding_number % 2 != 0 }
            FillRule::NonZero => { winding_number != 0 }
        }
    }

    fn transition(&self, prev_winding: i16, new_winding: i16) -> Transition {
        match (self.is_in(prev_winding), self.is_in(new_winding)) {
            (false, true) => Transition::In,
            (true, false) => Transition::Out,
            _ => Transition::None,
        }
    }

    fn update_winding(&self, winding: &mut WindingState, edge_winding: i16) {
        let prev_winding_number = winding.number;
        winding.number += edge_winding;
        winding.transition = self.transition(prev_winding_number, winding.number);
        if winding.transition == Transition::In {
            winding.span_index += 1;
        }
    }
}

struct ActiveEdge {
    from: Point,
    to: Point,
    ctrl: Point,

    range_start: f32,

    winding: i16,
    is_merge: bool,
}

struct ActiveEdgeAux {
    from_id: EndpointId,
    ctrl_id: CtrlPointId,
    to_id: EndpointId,
    upper_vertex: VertexId,
}

struct ActiveEdges {
    edges: Vec<ActiveEdge>,
    aux: Vec<ActiveEdgeAux>,
}

type SpanIdx = i32;

struct Spans {
    spans: Vec<MonotoneTessellator>,
}

impl Spans {
    fn begin_span(&mut self, span_idx: SpanIdx, position: &Point, vertex: VertexId) {
        let idx = span_idx as usize;
        self.spans.insert(
            idx,
            MonotoneTessellator::new().begin(*position, vertex)
        );
    }

    fn end_span(
        &mut self,
        span_idx: SpanIdx,
        position: &Point,
        id: VertexId,
        output: &mut GeometryBuilder<Vertex>,
    ) {
        let idx = span_idx as usize;

        {
            let tess = &mut self.spans[idx];
            tess.end(*position, id);
            tess.flush(output);
        }

        self.spans.remove(idx);
    }

    fn split_span(
        &mut self,
        span_idx: SpanIdx,
        split_position: &Point,
        split_id: VertexId,
        a_position: &Point,
        a_id: VertexId
    ) {
        let idx = span_idx as usize;

        //        /....
        // a --> x.....
        //      /.\....
        //     /...x... <-- current split vertex
        //    /.../ \..

        self.spans.insert(
            idx,
            MonotoneTessellator::new().begin(*a_position, a_id)
        );

        self.spans[idx].vertex(*split_position, split_id, Side::Right);
        self.spans[idx + 1].vertex(*split_position, split_id, Side::Left);
    }

    fn merge_spans(
        &mut self,
        span_idx: SpanIdx,
        current_position: &Point,
        current_vertex: VertexId,
        merge_position: &Point,
        merge_vertex: VertexId,
        output: &mut GeometryBuilder<Vertex>,
    ) {
        //  \...\ /.
        //   \...x..  <-- merge vertex
        //    \./...  <-- active_edge
        //     x....  <-- current vertex

        let idx = span_idx as usize;
        self.spans[idx].vertex(
            *merge_position,
            merge_vertex,
            Side::Right,
        );

        self.spans[idx + 1].vertex(
            *merge_position,
            merge_vertex,
            Side::Left,
        );

        self.end_span(
            span_idx,
            current_position,
            current_vertex,
            output,
        );
    }
}

struct PendingEdge {
    from: Point,
    to: Point,
    ctrl: Point,

    range_start: f32,
    angle: f32,

    from_id: EndpointId,
    ctrl_id: CtrlPointId,
    to_id: EndpointId,

    upper_vertex: VertexId,

    winding: i16,
}

impl ActiveEdge {
    fn solve_x_for_y(&self, y: f32) -> f32 {
        // TODO: curves.
        LineSegment {
            from: self.from,
            to: self.to,
        }.solve_x_for_y(y)
    }
}

impl FillTessellator {
    pub fn new() -> Self {
        FillTessellator {
            active: ActiveEdges {
                edges: Vec::new(),
                aux: Vec::new(),
            },
            pending_edges: Vec::new(),
            fill_rule: FillRule::EvenOdd,
            events: Vec::new(),
            fill: Spans {
                spans: Vec::new(),
            }
        }
    }

    pub fn tessellate_path(
        &mut self,
        path: &Path,
        options: &FillOptions,
        builder: &mut GeometryBuilder<Vertex>
    ) {
        self.fill_rule = options.fill_rule;

        path.sort(&mut self.events);

        builder.begin_geometry();

        self.tessellator_loop(path, builder);

        builder.end_geometry();

        assert!(self.active.edges.is_empty());
        assert!(self.fill.spans.is_empty());

        println!("\n ***************** \n");
    }

    fn tessellator_loop(&mut self, path: &Path, output: &mut GeometryBuilder<Vertex>) {
        while let Some(event) = self.events.pop() {
            let segment_id_a = event.segment;
            let current_endpoint = path.segment_from(segment_id_a);
            let segment_id_b = path.previous_segment(segment_id_a);
            let endpoint_id_b = path.segment_from(segment_id_b);
            let endpoint_id_a = path.segment_to(segment_id_a);

            let endpoint_pos_a = path.endpoint(endpoint_id_a);
            let endpoint_pos_b = path.endpoint(endpoint_id_b);
            let current_pos = path.endpoint(current_endpoint);

            let after_a = is_after(current_pos, endpoint_pos_a);
            let after_b = is_after(current_pos, endpoint_pos_b);

            let mut edges_above = 0;
            let vertex_id = output.add_vertex(current_pos);

            if after_a {
                edges_above += 1;
            } else {
                let ctrl_id_a = path.segment_ctrl(segment_id_a);
                self.pending_edges.push(PendingEdge {
                    from: current_pos,
                    ctrl: path.ctrl_point(ctrl_id_a),
                    to: endpoint_pos_a,

                    range_start: 0.0,
                    angle: (endpoint_pos_a - current_pos).angle_from_x_axis().radians,

                    from_id: current_endpoint,
                    ctrl_id: ctrl_id_a,
                    to_id: endpoint_id_a,

                    upper_vertex: vertex_id,

                    winding: 1,
                });
            }

            if after_b {
                edges_above += 1;
            } else {
                let ctrl_id_b = path.segment_ctrl(segment_id_b);
                self.pending_edges.push(PendingEdge {
                    from: current_pos,
                    ctrl: path.ctrl_point(ctrl_id_b),
                    to: endpoint_pos_b,

                    range_start: 0.0,
                    angle: (endpoint_pos_b - current_pos).angle_from_x_axis().radians,

                    from_id: current_endpoint,
                    ctrl_id: ctrl_id_b,
                    to_id: endpoint_id_b,

                    upper_vertex: vertex_id,

                    winding: -1,
                });
            }

            self.process_events(
                current_pos,
                vertex_id,
                current_endpoint,
                edges_above,
                output,
            );
        }
    }

    fn process_events(
        &mut self,
        current_pos: Point,
        current_vertex: VertexId,
        current_endpoint: EndpointId,
        edges_above: u32,
        output: &mut GeometryBuilder<Vertex>,
    ) {
        println!("");
        println!(" --- events at [{}, {}]                       {} -> {}",
            current_pos.x, current_pos.y,
            edges_above, self.pending_edges.len(),
        );

        // The span index starts at -1 so that entering the first span (of index 0) increments
        // it to zero.
        let mut winding = WindingState {
            span_index: -1,
            number: 0,
            transition: Transition::None,
        };
        let mut winding_below = None;
        let mut above = self.active.edges.len()..self.active.edges.len();
        let mut connecting_edges = false;
        let mut first_transition_above = true;
        let mut pending_merge = None;
        let mut pending_right = None;
        let mut prev_transition_in = None;

        // First go through the sweep line and visit all edges that end at the
        // current position.

        for (i, active_edge) in self.active.edges.iter_mut().enumerate() {
            if active_edge.is_merge && connecting_edges {
                // TODO: It might be more robust to defer resolving merges to
                // after the loop in order to separate the analysis of the sweep
                // line and the mutations into two phases.
                // However we would have to be careful about either resolving
                // merges as the last step (if possible) or making sure we don't
                // invalidate the indices of other postponed events on the right
                // side of the merge.

                println!(" Resolve merge event {} at {:?} ending span {}", i, active_edge.to, winding.span_index);

                //  \...\ /.
                //   \...x..  <-- merge vertex
                //    \./...  <-- active_edge
                //     x....  <-- current vertex

                let merge_vertex = self.active.aux[i].upper_vertex;
                let merge_position = active_edge.from;
                self.fill.merge_spans(
                    winding.span_index,
                    &current_pos,
                    current_vertex,
                    &merge_position,
                    merge_vertex,
                    output,
                );

                // Fix up the active edge.
                // It's not 100% clear whether we need to do all of this, other
                // than:
                // - removing the merge flag,
                // - having the edge end at the current position,
                // - setting the winding to zero,
                // since ideally the rest of the loop shouldn't use this edge
                // and it will get removed after the loop.

                let from_id = self.active.aux[i].from_id;
                self.active.aux[i] = ActiveEdgeAux {
                    from_id,
                    ctrl_id: CtrlPointId::INVALID,
                    to_id: current_endpoint,
                    upper_vertex: current_vertex,
                };
                *active_edge = ActiveEdge {
                    from: merge_position,
                    ctrl: point(f32::NAN, f32::NAN),
                    range_start: 0.0,
                    to: current_pos,
                    is_merge: false,
                    // We enterred the left span (which we just removed), and
                    // are we now in the right span, so make sure this edge does
                    // not generate a transition by giving it a winding of zero.
                    winding: 0,
                };
            }

            let was_connecting_edges = connecting_edges;

            if points_are_equal(current_pos, active_edge.to) {
                if !connecting_edges {
                    debug_assert!(edges_above != 0);
                    connecting_edges = true;
                }
            } else {
                let ex = active_edge.solve_x_for_y(current_pos.y);
                println!("ex: {}", ex);

                if ex == current_pos.x {
                    //connecting_edges = true;
                    // TODO
                    //unimplemented!();
                    println!(" -- vertex on an edge!");
                }

                if ex > current_pos.x {
                    above.end = i;
                    break;
                }
            }

            if active_edge.is_merge {
                // \.....\ /...../
                //  \.....x...../   <--- merge vertex
                //   \....:..../
                // ---\---:---/----  <-- sweep line
                //     \..:../

                println!(" bump span idx because of merge vertex");

                // An unresolved merge vertex implies the left and right spans are
                // adjacent and there is no transition between the two which means
                // we need to bump the span index manually.
                winding.span_index += 1;

                // If we are connecting we should have already resolved this merge
                // event.
                debug_assert!(!connecting_edges);

                // This is unnecessary assuming the assertion above.
                winding.transition = Transition::None;
            } else {
                // (common case)
                self.fill_rule.update_winding(&mut winding, active_edge.winding);
            }

            if !was_connecting_edges && connecting_edges {
                // We just started connecting edges above the current point.
                // Remember the current winding state because this is what we will
                // start from when handling the pending edges below the current point.
                winding_below = Some(winding.clone());
                above.start = i;
            }

            if !connecting_edges {
                continue;
            }

            println!("{:?}", winding.transition);

            match winding.transition {
                Transition::Out => {
                    if first_transition_above {
                        if self.pending_edges.is_empty() {
                            // Merge event.
                            pending_merge = Some(i);
                        } else {
                            // Right event.
                            pending_right = Some(i);
                        }
                    } else if let Some(in_idx) = prev_transition_in.take() {
                        // TODO: (same comment as the merge resolve step) It might
                        // be more robust to defer ending spans after the loop in
                        // order to separate the analysis of the sweep line and the
                        // mutations into two phases.

                        println!(" ** end ** edges: [{}, {}] span: {}",
                            in_idx, i,
                            winding.span_index
                        );

                        let position = current_pos;
                        self.fill.end_span(
                            winding.span_index,
                            &position,
                            current_vertex,
                            output,
                        );
                        // Account for the span we just removed.
                        winding.span_index -= 1;
                    }
                }
                Transition::In => {
                    prev_transition_in = Some(i)
                }
                Transition::None => {}
            }

            if winding.transition != Transition::None {
                first_transition_above = false;
            }
        }

        // Fix up above index range in case there was no connecting edges.
        above.start = usize::min(above.start, above.end);

        println!("connecting edges: {}..{}", above.start, above.end);

        let mut winding = winding_below.unwrap_or(winding);
        let mut prev_transition_in = None;

        self.sort_ending_edges();

        if let Some(idx) = pending_right {
            // Right event.
            //
            //  ..../
            //  ...x
            //  ....\
            //

            println!(" ** right ** edge: {} span: {}", idx, winding.span_index);

            self.fill.spans[winding.span_index as usize].vertex(
                current_pos,
                current_vertex,
                Side::Right,
            );
        } else if let Some(in_idx) = pending_merge {
            // Merge event.
            //
            //  ...\   /...
            //  ....\ /....
            //  .....x.....
            //

            println!(" ** merge ** edges: [{}, {}] span: {}",
                in_idx, above.end - 1,
                winding.span_index
            );

            let e = &mut self.active.edges[in_idx];

            e.is_merge = true;
            e.from = e.to;
            e.ctrl = e.to;
            e.winding = 0;

            let aux = &mut self.active.aux[in_idx];
            aux.from_id = aux.to_id;
            aux.ctrl_id = CtrlPointId::INVALID;
            aux.upper_vertex = current_vertex;

        } else if winding.transition == Transition::In && self.pending_edges.len() % 2 == 1 {
            // Left event.
            //
            //     /...
            //    x....
            //     \...
            //

            println!(" ** left ** edge {} span: {}", above.start, winding.span_index);

            self.fill.spans[winding.span_index as usize].vertex(
                current_pos,
                current_vertex,
                Side::Left,
            );
        }

        if self.fill_rule.is_in(winding.number)
            && above.start == above.end
            && self.pending_edges.len() >= 2 {

            // Split event.
            //
            //  ...........
            //  .....x.....
            //  ..../ \....
            //  .../   \...
            //

            println!(" ** split ** edge {} span: {}", above.start, winding.span_index);
            let edge_above = above.start - 1;

            let upper_pos = self.active.edges[edge_above].from;
            let upper_id = self.active.aux[edge_above].upper_vertex;

            if self.active.edges[edge_above].is_merge {

                // Split vertex under a merge vertex
                //
                //  ...\ /...
                //  ....x....   <-- merge vertex (upper)
                //  ....:....
                //  ....x....   <-- current split vertex
                //  .../ \...
                //
                println!("   -> merge+split");
                let span_index = winding.span_index as usize;

                self.fill.spans[span_index - 1].vertex(
                    upper_pos,
                    upper_id,
                    Side::Right,
                );
                self.fill.spans[span_index - 1].vertex(
                    current_pos,
                    current_vertex,
                    Side::Right,
                );

                self.fill.spans[span_index].vertex(
                    upper_pos,
                    upper_id,
                    Side::Left,
                );
                self.fill.spans[span_index].vertex(
                    current_pos,
                    current_vertex,
                    Side::Left,
                );

                self.active.edges.remove(edge_above);
                above.start -= 1;
                above.end -= 1;
            } else {
                self.fill.split_span(
                    winding.span_index,
                    &current_pos,
                    current_vertex,
                    &upper_pos,
                    upper_id,
                );
            }

            winding.span_index += 1;
        }

        // Go through the edges starting at the current point and emmit
        // start events.

        for (i, pending_edge) in self.pending_edges.iter().enumerate() {
            self.fill_rule.update_winding(&mut winding, pending_edge.winding);

            match winding.transition {
                Transition::In => {
                    prev_transition_in = Some(i);
                }
                Transition::Out => {
                    if let Some(in_idx) = prev_transition_in {

                        println!(" ** start ** edges: [{}, {}] span: {}", in_idx, i, winding.span_index);

                        // Start event.
                        //
                        //      x
                        //     /.\
                        //    /...\
                        //

                        // TODO: if this is an intersection we must create a vertex
                        // and use it instead of the upper endpoint of the edge.
                        let vertex = self.pending_edges[in_idx].upper_vertex;
                        println!(" begin span {} ({})", winding.span_index, self.fill.spans.len());
                        self.fill.begin_span(
                            winding.span_index,
                            &current_pos,
                            vertex
                        );
                    }
                }
                Transition::None => {}
            }
        }

        self.update_active_edges(above);

        println!("sweep line: {}", self.active.edges.len());
        for e in &self.active.edges {
            if e.is_merge {
                println!("| (merge) {}", e.from);
            } else {
                println!("| {} -> {}", e.from, e.to);
            }
        }
        println!("spans: {}", self.fill.spans.len());
    }

    fn update_active_edges(&mut self, above: Range<usize>) {
        // Remove all edges from the "above" range except merge
        // vertices.
        println!(" remove {} edges ({}..{})", above.end - above.start, above.start, above.end);
        let mut rm_index = above.start;
        for _ in 0..(above.end - above.start) {
            if self.active.edges[rm_index].is_merge {
                rm_index += 1
            } else {
                self.active.edges.remove(rm_index);
                self.active.aux.remove(rm_index);
            }
        }

        // Insert the pending edges.

        let first_edge_below = above.start;
        for (i, edge) in self.pending_edges.drain(..).enumerate() {
            let idx = first_edge_below + i;

            self.active.edges.insert(idx, ActiveEdge {
                from: edge.from,
                to: edge.to,
                ctrl: edge.ctrl,
                range_start: edge.range_start,
                winding: edge.winding,
                is_merge: false,
            });

            self.active.aux.insert(idx, ActiveEdgeAux {
                from_id: edge.from_id,
                to_id: edge.to_id,
                ctrl_id: edge.ctrl_id,
                upper_vertex: edge.upper_vertex,
            });
        }
    }

    fn sort_ending_edges(&mut self) {
        self.pending_edges.sort_by(|a, b| {
            b.angle.partial_cmp(&a.angle).unwrap_or(Ordering::Equal)
        });
    }
}

fn points_are_equal(a: Point, b: Point) -> bool {
    // TODO: Use the tolerance threshold.
    a == b
}


fn compare_positions(a: Point, b: Point) -> Ordering {
    if a.y > b.y {
        return Ordering::Greater;
    }
    if a.y < b.y {
        return Ordering::Less;
    }
    if a.x > b.x {
        return Ordering::Greater;
    }
    if a.x < b.x {
        return Ordering::Less;
    }
    return Ordering::Equal;
}

#[inline]
fn is_after(a: Point, b: Point) -> bool {
    a.y > b.y || (a.y == b.y && a.x > b.x)
}

#[cfg(test)]
use geometry_builder::{VertexBuffers, simple_builder};

#[test]
fn new_tess1() {
    println!("");

    let mut builder = Builder::new();
    builder.move_to(point(0.0, 0.0));
    builder.line_to(point(5.0, -5.0));
    builder.line_to(point(10.0, 0.0));
    builder.line_to(point(9.0, 5.0));
    builder.line_to(point(10.0, 10.0));
    builder.line_to(point(5.0, 6.0));
    builder.line_to(point(0.0, 10.0));
    builder.line_to(point(1.0, 5.0));
    builder.close();

    builder.move_to(point(20.0, -1.0));
    builder.line_to(point(25.0, 1.0));
    builder.line_to(point(25.0, 9.0));
    builder.close();


    let path = builder.build();

    let mut tess = FillTessellator::new();

    let mut buffers: VertexBuffers<Vertex> = VertexBuffers::new();

    tess.tessellate_path(
        &path,
        &FillOptions::default(),
        &mut simple_builder(&mut buffers),
    );

    panic!();
}

#[test]
fn new_tess_merge() {
    println!("");

    let mut builder = Builder::new();
    builder.move_to(point(0.0, 0.0));  // start
    builder.line_to(point(5.0, 5.0));  // merge
    builder.line_to(point(5.0, 1.0));  // start
    builder.line_to(point(10.0, 6.0)); // merge
    builder.line_to(point(11.0, 2.0)); // start
    builder.line_to(point(11.0, 10.0));// end
    builder.line_to(point(0.0, 9.0));  // left
    builder.close();

    let path = builder.build();

    let mut tess = FillTessellator::new();

    let mut buffers: VertexBuffers<Vertex> = VertexBuffers::new();

    tess.tessellate_path(
        &path,
        &FillOptions::default(),
        &mut simple_builder(&mut buffers),
    );

    panic!();
}

// "m 85.728423,257.84471 -21.7607,-11.36642 85.533557,82.88722 80.86714,-82.2058 z m 60.609997,8.9 12.8,13.14 h -26.33 l 13.55,-13.16 z"
