from pathlib import Path

response = Path("src/rotating_contact_response.rs")
text = response.read_text()

old = '''pub struct RotatingContactResponseScratch3d {
    indices: BTreeMap<BodyId, usize>,
    resolved_indices: Vec<Option<(usize, usize)>>,
'''
new = '''pub struct RotatingContactResponseScratch3d {
    indices: BTreeMap<BodyId, usize>,
    indexed_body_ids: Vec<BodyId>,
    resolved_indices: Vec<Option<(usize, usize)>>,
'''
if text.count(old) != 1:
    raise RuntimeError("scratch struct marker not found")
text = text.replace(old, new, 1)

marker = '''}

/// Resolves every contact in one shared sampled frontier with bounded simultaneous passes.
'''
impl_block = '''}

impl RotatingContactResponseScratch3d {
    pub(crate) fn ensure_body_index(&mut self, boxes: &[RigidBox3d]) {
        let layout_matches = self.indexed_body_ids.len() == boxes.len()
            && self
                .indexed_body_ids
                .iter()
                .zip(boxes)
                .all(|(id, rigid_box)| *id == rigid_box.body().id());
        if layout_matches {
            return;
        }

        self.indices.clear();
        self.indexed_body_ids.clear();
        self.indexed_body_ids.reserve(boxes.len());
        for (index, rigid_box) in boxes.iter().enumerate() {
            let id = rigid_box.body().id();
            self.indices.insert(id, index);
            self.indexed_body_ids.push(id);
        }
    }

    pub(crate) fn indexed_box<'a>(
        &self,
        boxes: &'a [RigidBox3d],
        id: BodyId,
    ) -> Option<&'a RigidBox3d> {
        let index = *self.indices.get(&id)?;
        boxes
            .get(index)
            .filter(|rigid_box| rigid_box.body().id() == id)
    }
}

/// Resolves every contact in one shared sampled frontier with bounded simultaneous passes.
'''
if text.count(marker) != 1:
    raise RuntimeError("scratch impl insertion marker not found")
text = text.replace(marker, impl_block, 1)

old = '''    let RotatingContactResponseScratch3d {
        indices,
        resolved_indices,
'''
new = '''    scratch.ensure_body_index(&frontier.boxes);
    let RotatingContactResponseScratch3d {
        indices,
        indexed_body_ids: _,
        resolved_indices,
'''
if text.count(old) != 1:
    raise RuntimeError("scratch destructure marker not found")
text = text.replace(old, new, 1)

old = '''    indices.clear();
    indices.extend(
        frontier
            .boxes
            .iter()
            .enumerate()
            .map(|(index, rigid_box)| (rigid_box.body.id, index)),
    );
    let mut boxes = frontier.boxes;
'''
new = '''    let mut boxes = frontier.boxes;
'''
if text.count(old) != 1:
    raise RuntimeError("per-response index rebuild marker not found")
text = text.replace(old, new, 1)

old = '''    combined.clear();
    combined.reserve(boxes.len().saturating_sub(combined.capacity()));
'''
new = '''    combined.clear();
    combined.reserve(boxes.len());
'''
if text.count(old) != 1:
    raise RuntimeError("combined reserve marker not found")
text = text.replace(old, new, 1)
response.write_text(text)

repeated = Path("src/repeated_rotating_events.rs")
text = repeated.read_text()

old = '''        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &active,
            &mut contacts,
            broad_phase,
        )?;
'''
new = '''        let current = refresh_current_contacts_for_changed_bodies(
            &boxes,
            &active,
            &mut contacts,
            broad_phase,
            response_scratch,
        )?;
'''
if text.count(old) != 1:
    raise RuntimeError("refresh call marker not found")
text = text.replace(old, new, 1)

old = '''fn refresh_current_contacts_for_changed_bodies(
    boxes: &[RigidBox3d],
    active: &[crate::BodyId],
    contacts: &mut BTreeMap<crate::RotationalSweepPair3d, RotatingContactSearchHit3d>,
    broad_phase: &mut RotatingBroadPhase3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
'''
new = '''fn refresh_current_contacts_for_changed_bodies(
    boxes: &[RigidBox3d],
    active: &[crate::BodyId],
    contacts: &mut BTreeMap<crate::RotationalSweepPair3d, RotatingContactSearchHit3d>,
    broad_phase: &mut RotatingBroadPhase3d,
    body_index: &RotatingContactResponseScratch3d,
) -> Result<CurrentContactFrontierResult3d, RotatingContactFrontierError3d> {
'''
if text.count(old) != 1:
    raise RuntimeError("refresh signature marker not found")
text = text.replace(old, new, 1)

old = '''    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
'''
new = '''    let mut active_boxes = Vec::with_capacity(active.len());
    for id in active {
        let rigid_box = body_index
            .indexed_box(boxes, *id)
            .ok_or(RotatingContactFrontierError3d::MissingBody(*id))?;
        active_boxes.push(rigid_box);
    }
'''
if text.count(old) != 1:
    raise RuntimeError("active full-scan marker not found")
text = text.replace(old, new, 1)

old = '''    for pair in candidates {
        let left = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right = boxes
            .iter()
            .find(|rigid_box| rigid_box.body().id() == pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
'''
new = '''    for pair in candidates {
        let left = body_index
            .indexed_box(boxes, pair.left)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.left))?;
        let right = body_index
            .indexed_box(boxes, pair.right)
            .ok_or(RotatingContactFrontierError3d::MissingBody(pair.right))?;
'''
if text.count(old) != 1:
    raise RuntimeError("candidate full-scan marker not found")
text = text.replace(old, new, 1)

old = '''    let mut contacts = BTreeMap::new();
    Ok(refresh_current_contacts_for_changed_bodies(
        boxes,
        &active,
        &mut contacts,
        &mut broad_phase,
    )?
    .frontier)
'''
new = '''    let mut contacts = BTreeMap::new();
    let mut body_index = RotatingContactResponseScratch3d::default();
    body_index.ensure_body_index(boxes);
    Ok(refresh_current_contacts_for_changed_bodies(
        boxes,
        &active,
        &mut contacts,
        &mut broad_phase,
        &body_index,
    )?
    .frontier)
'''
if text.count(old) != 1:
    raise RuntimeError("test helper refresh marker not found")
text = text.replace(old, new, 1)
repeated.write_text(text)
