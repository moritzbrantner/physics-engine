// Native comparison controls must keep their own keyboard behavior instead of forwarding
// arrows/Space to the document-level game input handler. Do not cancel the browser default.
for (const control of document.querySelectorAll(
  "#character-mode, #upright-crates, #fixed-geometry-mode",
)) {
  control.addEventListener("keydown", (event) => event.stopPropagation());
}
