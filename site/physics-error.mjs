const DETAIL_LABELS = new Map([
  [601, "persistent-tail resolution limit"],
  [602, "persistent-tail unsafe motion"],
  [603, "persistent-tail arithmetic overflow"],
  [610, "repeated-event failure"],
  [611, "repeated-event limit"],
  [612, "remaining-time numerical range"],
  [613, "invalid repeated-event remainder"],
  [614, "contact-frontier failure"],
  [615, "repeated-event response failure"],
  [620, "free-flight failure"],
  [630, "contact-geometry failure"],
  [640, "contact-response failure"],
  [699, "other rotating-world failure"],
]);

export function physicsFailureMessage(error, detail = 0) {
  const reset = "Reset to start from the deterministic fixture again.";
  if (error !== 6 || detail === 0) {
    return `Physics stopped fail-closed with sandbox error ${error}. ${reset}`;
  }

  const label = DETAIL_LABELS.get(detail) ?? "unknown rotating-world failure";
  return `Physics stopped fail-closed with sandbox error 6 / detail ${detail} (${label}). ${reset}`;
}
