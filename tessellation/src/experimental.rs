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
use std::{u16, u32, f32};


#[cfg(feature="debugger")]
use debugger::*;
#[cfg(feature="debugger")]
use path_fill::dbg;

pub type Vertex = Point;

macro_rules! tess_log {
    ($obj:ident, $fmt:expr) => (
        if $obj.log {
            println!($fmt);
        }
    );
    ($obj:ident, $fmt:expr, $($arg:tt)*) => (
        if $obj.log {
            println!($fmt, $($arg)*);
        }
    );
}

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
            sub_paths: vec![
                SubPathInfo {
                    range: 0..0,
                    is_closed: true,
                }
            ],
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
                self.endpoints[a.vertex.0 as usize],
                self.endpoints[b.vertex.0 as usize],
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
    tolerance: f32,
    in_sub_path: bool,
}

impl Builder {
    pub fn new() -> Self {
        Builder::with_tolerance(FillOptions::DEFAULT_TOLERANCE)
    }

    pub fn with_tolerance(tolerance: f32) -> Self {
        Builder {
            path: Path::new(),
            tolerance,
            in_sub_path: false,
        }
    }

    pub fn line_to(&mut self, to_pos: Point) -> bool {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        if self.segment_is_too_small(&to_pos) {
            return false;
        }

        self.in_sub_path = true;
        let from = EndpointId((self.path.endpoints.len() - 1) as u16);
        let ctrl = CtrlPointId::INVALID;
        let to = EndpointId(from.0 + 1);

        self.path.endpoints.push(to_pos);

        self.path.segments.push(Segment { from, ctrl, to });

        true
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

        self.in_sub_path = true;
        QuadraticBezierSegment {
            from: *self.path.endpoints.last().unwrap(),
            ctrl: ctrl_pos,
            to: to_pos,
        }.for_each_monotonic(&mut |monotonic| {
            self.monotonic_quadratic_bezier_to(
                monotonic.segment().from,
                monotonic.segment().to
            );
        });
    }

    pub fn cubic_bezier_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        if self.path.endpoints.is_empty() {
            self.path.endpoints.push(point(0.0, 0.0));
        }

        self.in_sub_path = true;

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

        self.in_sub_path = true;
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

    fn segment_is_too_small(&self, to: &Point) -> bool {
        (*self.path.endpoints.last().unwrap() - *to).square_length() < self.tolerance * self.tolerance
    }

    fn monotonic_quadratic_bezier_to(&mut self, ctrl_pos: Point, to_pos: Point) {
        if self.segment_is_too_small(&to_pos) {
            return;
        }
        let from = EndpointId((self.path.endpoints.len() - 1) as u16);
        let to = EndpointId(from.0 + 1);
        self.path.endpoints.push(to_pos);

        let ctrl = CtrlPointId((self.path.ctrl_points.len() - 1) as u16);
        self.path.ctrl_points.push(ctrl_pos);

        self.path.segments.push(Segment{ from, ctrl, to });
    }

    fn end_sub_path(&mut self, is_closed: bool) {
        if !self.in_sub_path {
            return;
        }
        let mut sp_end = self.path.segments.len();
        let sp_start = self.path.sub_paths.last()
            .map(|sp| sp.range.start)
            .unwrap();
        if sp_end > sp_start {
            if is_closed && !self.path.endpoints.is_empty() {
                let first = self.path.segments[sp_start].from.0 as usize;
                let first_point = self.path.endpoints[first];
                if self.line_to(first_point) {
                    sp_end += 1;
                }
            }

            *self.path.sub_paths.last_mut().unwrap() = SubPathInfo {
                range: sp_start..sp_end,
                is_closed,
            };
            self.path.sub_paths.push(SubPathInfo {
                range: sp_end..sp_end,
                is_closed,
            });
        }

        self.in_sub_path = false;
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
    current_position: Point,
    active: ActiveEdges,
    edges_below: Vec<PendingEdge>,
    fill_rule: FillRule,
    events: Vec<Event>,
    fill: Spans,
    log: bool,

    #[cfg(feature="debugger")]
    debugger: Option<Box<dyn Debugger2D>>,
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

struct Span {
    tess: MonotoneTessellator,
    remove: bool,
}

struct Spans {
    spans: Vec<Span>,
}

impl Spans {
    fn begin_span(&mut self, span_idx: SpanIdx, position: &Point, vertex: VertexId) {
        let idx = span_idx as usize;
        self.spans.insert(
            idx,
            Span {
                tess: MonotoneTessellator::new().begin(*position, vertex),
                remove: false,
            }
        );
    }

    fn end_span(
        &mut self,
        span_idx: SpanIdx,
        position: &Point,
        id: VertexId,
        output: &mut dyn GeometryBuilder<Vertex>,
    ) {
        let idx = span_idx as usize;

        let span = &mut self.spans[idx];
        span.remove = true;
        span.tess.end(*position, id);
        span.tess.flush_experimental(output);
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
            Span {
                tess: MonotoneTessellator::new().begin(*a_position, a_id),
                remove: false,
            }
        );

        self.spans[idx].tess.vertex(*split_position, split_id, Side::Right);
        self.spans[idx + 1].tess.vertex(*split_position, split_id, Side::Left);
    }

    fn merge_spans(
        &mut self,
        span_idx: SpanIdx,
        current_position: &Point,
        current_vertex: VertexId,
        merge_position: &Point,
        merge_vertex: VertexId,
        output: &mut dyn GeometryBuilder<Vertex>,
    ) {
        //  \...\ /.
        //   \...x..  <-- merge vertex
        //    \./...  <-- active_edge
        //     x....  <-- current vertex

        let idx = span_idx as usize;
        if self.spans.len() <= idx + 1 {
            // TODO: we can only run into this if the order of the sweep line
            // is invalid. Need to re-sort it.
            return
        }

        self.spans[idx].tess.vertex(
            *merge_position,
            merge_vertex,
            Side::Right,
        );

        self.spans[idx + 1].tess.vertex(
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

    fn cleanup_spans(&mut self) {
        // Get rid of the spans that were marked for removal.
        self.spans.retain(|span|{ !span.remove });
    }
}

struct PendingEdge {
    from: Point, // TODO: unnecessary since this is always the current position
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
            current_position: point(f32::MIN, f32::MIN),
            active: ActiveEdges {
                edges: Vec::new(),
                aux: Vec::new(),
            },
            edges_below: Vec::new(),
            fill_rule: FillRule::EvenOdd,
            events: Vec::new(),
            fill: Spans {
                spans: Vec::new(),
            },
            log: true,

            #[cfg(feature="debugger")]
            debugger: None,
        }
    }

    pub fn tessellate_path(
        &mut self,
        path: &Path,
        options: &FillOptions,
        builder: &mut dyn GeometryBuilder<Vertex>
    ) {
        self.fill_rule = options.fill_rule;

        path.sort(&mut self.events);

        builder.begin_geometry();

        self.tessellator_loop(path, builder);

        builder.end_geometry();

        assert!(self.active.edges.is_empty());
        assert!(self.fill.spans.is_empty());

        tess_log!(self, "\n ***************** \n");
    }

    pub fn enable_logging(&mut self) {
        self.log = true;
    }

    fn tessellator_loop(&mut self, path: &Path, output: &mut dyn GeometryBuilder<Vertex>) {
        self.current_position = point(f32::MIN, f32::MIN);
        let events = mem::replace(&mut self.events, Vec::new());
        let mut event_iter = events.iter();
        let mut next_event = event_iter.next();
        loop {
            let mut edges_above = 0;
            let mut next_position = None;
            let mut current_endpoint = EndpointId(u16::MAX);
            let mut vertex_id = VertexId(u32::MAX);
            while let Some(event) = next_event {
                let segment_id_a = event.segment;
                current_endpoint = path.segment_from(segment_id_a);
                let current_pos = path.endpoint(current_endpoint);
                if current_pos == self.current_position {
                    next_event = event_iter.next();
                    let segment_id_b = path.previous_segment(segment_id_a);
                    let endpoint_id_b = path.segment_from(segment_id_b);
                    let endpoint_id_a = path.segment_to(segment_id_a);

                    let endpoint_pos_a = path.endpoint(endpoint_id_a);
                    let endpoint_pos_b = path.endpoint(endpoint_id_b);

                    let after_a = is_after(current_pos, endpoint_pos_a);
                    let after_b = is_after(current_pos, endpoint_pos_b);

                    vertex_id = output.add_vertex(current_pos);

                    if after_a {
                        edges_above += 1;
                    } else {
                        let ctrl_id_a = path.segment_ctrl(segment_id_a);
                        self.edges_below.push(PendingEdge {
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
                        self.edges_below.push(PendingEdge {
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
                } else {
                    next_position = Some(current_pos);
                    break;
                }
            }

            if edges_above > 0 || self.edges_below.len() > 0 {
                self.process_events(
                    vertex_id,
                    current_endpoint,
                    edges_above,
                    output,
                );
            }

            if let Some(position) = next_position {
                self.current_position = position;
            } else {
                break;
            }
        }
    }

    fn process_events(
        &mut self,
        current_vertex: VertexId,
        current_endpoint: EndpointId,
        edges_above: u32,
        output: &mut dyn GeometryBuilder<Vertex>,
    ) {
        tess_log!(self, "\n --- events at [{}, {}]                       {} -> {}",
            self.current_position.x, self.current_position.y,
            edges_above, self.edges_below.len(),
        );

        // The span index starts at -1 so that entering the first span (of index 0) increments
        // it to zero.
        let mut winding = WindingState {
            span_index: -1,
            number: 0,
            transition: Transition::None,
        };
        let mut winding_below: Option<WindingState> = None;
        let mut above = self.active.edges.len()..self.active.edges.len();
        let mut connecting_edges = false;
        let mut first_transition_above = true;
        let mut pending_merge = None;
        let mut pending_right = None;
        let mut prev_transition_in = None;

        let mut merges_to_resolve: Vec<(SpanIdx, usize)> = Vec::new();
        let mut spans_to_end = Vec::new();
        let mut edges_to_split = Vec::new();

        // First go through the sweep line and visit all edges that end at the
        // current position.

        for (i, active_edge) in self.active.edges.iter_mut().enumerate() {
            if active_edge.is_merge && connecting_edges {
                merges_to_resolve.push((winding.span_index, i));
                active_edge.to = self.current_position;
                // This is probably not necessary but it's confusing to have the two
                // not matching.
                self.active.aux[i].to_id = current_endpoint;

                // Bump the span index since the current span will be removed
                // later. If this is the index of the first touching span, then
                // we need to bump that one too.
                if let Some(ref mut below) = winding_below {
                    if below.span_index == winding.span_index {
                        below.span_index += 1;
                    }
                }
                winding.span_index += 1;
            }

            let was_connecting_edges = connecting_edges;

            if points_are_equal(self.current_position, active_edge.to) {
                if !connecting_edges {
                    debug_assert!(edges_above != 0);
                    connecting_edges = true;
                }
            } else {
                let ex = active_edge.solve_x_for_y(self.current_position.y);
                tess_log!(self, "ex: {}", ex);

                if ex == self.current_position.x {
                    tess_log!(self, " -- vertex on an edge!");
                    edges_to_split.push(i);

                    connecting_edges = true;
                }

                if ex > self.current_position.x {
                    above.end = i;
                    break;
                }
            }

            if connecting_edges || !active_edge.is_merge {
                // (common case)
                self.fill_rule.update_winding(&mut winding, active_edge.winding);
            } else {
                // \.....\ /...../
                //  \.....x...../   <--- merge vertex
                //   \....:..../
                // ---\---:---/----  <-- sweep line
                //     \..:../

                // An unresolved merge vertex implies the left and right spans are
                // adjacent and there is no transition between the two which means
                // we need to bump the span index manually.
                winding.span_index += 1;

                // If we were connecting we should have already resolved this merge
                // event.
                debug_assert!(!connecting_edges);
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

            tess_log!(self, "{:?}", winding.transition);

            match winding.transition {
                Transition::Out => {
                    if first_transition_above {
                        if self.edges_below.is_empty() {
                            // Merge event.
                            pending_merge = Some(i);
                        } else {
                            // Right event.
                            pending_right = Some(i);
                        }
                    } else if let Some(in_idx) = prev_transition_in.take() {
                        tess_log!(self, " ** end ** edges: [{}, {}] span: {}",
                            in_idx, i,
                            winding.span_index
                        );

                        if winding.span_index < self.fill.spans.len() as i32 {
                            spans_to_end.push(winding.span_index);
                            winding.span_index += 1; // not sure
                        } else {
                            // error!
                        }
                    }
                }
                Transition::In => {
                    prev_transition_in = Some(i);
                }
                Transition::None => {}
            }

            if winding.transition != Transition::None {
                first_transition_above = false;
            }
        }

        for (span_index, i) in merges_to_resolve {
            //  \...\ /.
            //   \...x..  <-- merge vertex
            //    \./...  <-- active_edge
            //     x....  <-- current vertex
            let active_edge: &mut ActiveEdge = &mut self.active.edges[i];
            let merge_vertex: VertexId = self.active.aux[i].upper_vertex;
            let merge_position = active_edge.from;

            self.fill.merge_spans(
                span_index,
                &self.current_position,
                current_vertex,
                &merge_position,
                merge_vertex,
                output,
            );

            active_edge.is_merge = false;

            tess_log!(self, " Resolve merge event {} at {:?} ending span {}", i, active_edge.to, span_index);
            #[cfg(feature="debugger")]
            debugger_monotone_split(&self.debugger, &merge_position, &self.current_position);
        }

        for span_index in spans_to_end {
            self.fill.end_span(
                span_index,
                &self.current_position,
                current_vertex,
                output,
            );
        }

        for edge_idx in edges_to_split {
            let to = self.active.edges[edge_idx].to;
            self.edges_below.push(PendingEdge {
                from: self.current_position,
                ctrl: point(f32::NAN, f32::NAN),
                to,

                range_start: 0.0,
                angle: (to - self.current_position).angle_from_x_axis().radians,

                from_id: current_endpoint,
                ctrl_id: CtrlPointId(u16::MAX),
                to_id: self.active.aux[edge_idx].to_id,

                upper_vertex: current_vertex,
                winding: self.active.edges[edge_idx].winding,
            });

            self.active.edges[edge_idx].to = self.current_position;
            self.active.aux[edge_idx].to_id = current_endpoint;
        }

        // Fix up above index range in case there was no connecting edges.
        above.start = usize::min(above.start, above.end);

        tess_log!(self, "connecting edges: {}..{} {:?}", above.start, above.end, winding.transition);

        let mut winding = winding_below.unwrap_or(winding);
        let mut prev_transition_in = None;

        self.sort_edges_below();

        if let Some(idx) = pending_right {
            // Right event.
            //
            //  ..../
            //  ...x
            //  ....\
            //

            tess_log!(self, " ** right ** edge: {} span: {}", idx, winding.span_index);

            self.fill.spans[winding.span_index as usize].tess.vertex(
                self.current_position,
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

            tess_log!(self, " ** merge ** edges: [{}, {}] span: {}",
                in_idx, above.end - 1,
                winding.span_index
            );

            let active_edge = &mut self.active.edges[in_idx];
            active_edge.is_merge = true;
            active_edge.from = active_edge.to;
            active_edge.ctrl = active_edge.to;
            active_edge.winding = 0;

            let aux = &mut self.active.aux[in_idx];
            aux.from_id = aux.to_id;
            aux.ctrl_id = CtrlPointId::INVALID;
            aux.upper_vertex = current_vertex;

        } else if winding.transition == Transition::In && self.edges_below.len() % 2 == 1 {
            // TODO: cond self.edges_below.len() % 2 == 1 is wrong if we had a right event

            // Left event.
            //
            //     /...
            //    x....
            //     \...
            //

            tess_log!(self, " ** left ** edge {} span: {}", above.start, winding.span_index);

            self.fill.spans[winding.span_index as usize].tess.vertex(
                self.current_position,
                current_vertex,
                Side::Left,
            );
        }

        if self.fill_rule.is_in(winding.number)
            && above.start == above.end
            && self.edges_below.len() >= 2 {

            // Split event.
            //
            //  ...........
            //  .....x.....
            //  ..../ \....
            //  .../   \...
            //

            tess_log!(self, " ** split ** edge {} span: {}", above.start, winding.span_index);
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
                tess_log!(self, "   -> merge+split");
                let span_index = winding.span_index as usize;

                self.fill.spans[span_index - 1].tess.vertex(
                    upper_pos,
                    upper_id,
                    Side::Right,
                );
                self.fill.spans[span_index - 1].tess.vertex(
                    self.current_position,
                    current_vertex,
                    Side::Right,
                );

                self.fill.spans[span_index].tess.vertex(
                    upper_pos,
                    upper_id,
                    Side::Left,
                );
                self.fill.spans[span_index].tess.vertex(
                    self.current_position,
                    current_vertex,
                    Side::Left,
                );

                self.active.edges.remove(edge_above);
                above.start -= 1;
                above.end -= 1;
            } else {
                self.fill.split_span(
                    winding.span_index,
                    &self.current_position,
                    current_vertex,
                    &upper_pos,
                    upper_id,
                );
            }

            #[cfg(feature="debugger")]
            debugger_monotone_split(&self.debugger, &upper_pos, &self.current_position);

            winding.span_index += 1;
        }

        // Go through the edges starting at the current point and emit
        // start events.

        for (i, pending_edge) in self.edges_below.iter().enumerate() {
            self.fill_rule.update_winding(&mut winding, pending_edge.winding);

            match winding.transition {
                Transition::In => {
                    prev_transition_in = Some(i);
                }
                Transition::Out => {
                    if let Some(in_idx) = prev_transition_in {

                        tess_log!(self, " ** start ** edges: [{}, {}] span: {}", in_idx, i, winding.span_index);

                        // Start event.
                        //
                        //      x
                        //     /.\
                        //    /...\
                        //

                        // TODO: if this is an intersection we must create a vertex
                        // and use it instead of the upper endpoint of the edge.
                        let vertex = self.edges_below[in_idx].upper_vertex;
                        tess_log!(self, " begin span {} ({})", winding.span_index, self.fill.spans.len());
                        self.fill.begin_span(
                            winding.span_index,
                            &self.current_position,
                            vertex
                        );
                    }
                }
                Transition::None => {}
            }
        }

        self.fill.cleanup_spans();

        self.update_active_edges(above);

        tess_log!(self, "sweep line: {}", self.active.edges.len());
        for e in &self.active.edges {
            if e.is_merge {
                tess_log!(self, "| (merge) {}", e.from);
            } else {
                tess_log!(self, "| {} -> {}", e.from, e.to);
            }
        }
        tess_log!(self, "spans: {}", self.fill.spans.len());
    }

    fn update_active_edges(&mut self, above: Range<usize>) {
        // Remove all edges from the "above" range except merge
        // vertices.
        tess_log!(self, " remove {} edges ({}..{})", above.end - above.start, above.start, above.end);
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
        for (i, edge) in self.edges_below.drain(..).enumerate() {
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

    fn sort_edges_below(&mut self) {
        // TODO: we'll need a better criterion than the tangent angle with quadratic béziers.
        self.edges_below.sort_by(|a, b| {
            b.angle.partial_cmp(&a.angle).unwrap_or(Ordering::Equal)
        });
    }

    #[cfg(feature="debugger")]
    pub fn install_debugger(&mut self, dbg: Box<dyn Debugger2D>) {
        self.debugger = Some(dbg)
    }

}

#[cfg(feature="debugger")]
fn debugger_monotone_split(debugger: &Option<Box<dyn Debugger2D>>, a: &Point, b: &Point) {
    if let Some(ref dbg) = debugger {
        dbg.edge(a, b, DARK_RED, dbg::MONOTONE_SPLIT);
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

    let mut buffers: VertexBuffers<Vertex, u16> = VertexBuffers::new();

    tess.tessellate_path(
        &path,
        &FillOptions::default(),
        &mut simple_builder(&mut buffers),
    );
}

#[test]
fn new_tess_merge() {

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

    let mut buffers: VertexBuffers<Vertex, u16> = VertexBuffers::new();

    tess.tessellate_path(
        &path,
        &FillOptions::default(),
        &mut simple_builder(&mut buffers),
    );
}

// cargo run --features=experimental -- show "M 0 0 L 1 1 0 2 Z M 2 0 1 1 2 2 Z" --tessellator experimental -fs
