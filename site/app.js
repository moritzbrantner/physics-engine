const canvas = document.querySelector("#scene");
const context = canvas.getContext("2d");
const speed = document.querySelector("#speed");
const speedValue = document.querySelector("#speed-value");
const restitution = document.querySelector("#restitution");
const restitutionValue = document.querySelector("#restitution-value");
const restart = document.querySelector("#restart");
const status = document.querySelector("#status");

const wall = { x: 700, halfWidth: 12 };
const body = { x: 90, halfWidth: 18, velocity: Number(speed.value) };
let previousTimestamp = null;
let lastImpact = null;

function reset() {
  body.x = 90;
  body.velocity = Number(speed.value);
  previousTimestamp = null;
  lastImpact = null;
  status.textContent = "Following the body's swept path across each frame.";
}

function updateLabels() {
  speedValue.value = `${speed.value} units/s`;
  restitutionValue.value = (Number(restitution.value) / 1000).toFixed(2);
}

function step(dt) {
  const leftFace = wall.x - wall.halfWidth;
  const rightFace = body.x + body.halfWidth;
  const remainingGap = leftFace - rightFace;
  lastImpact = null;

  if (body.velocity > 0 && remainingGap >= 0) {
    const travel = body.velocity * dt;
    if (travel >= remainingGap) {
      const toi = remainingGap / body.velocity;
      const postImpact = dt - toi;
      body.x += body.velocity * toi;
      body.velocity = -body.velocity * (Number(restitution.value) / 1000);
      body.x += body.velocity * postImpact;
      lastImpact = toi;
      status.textContent = `Impact found ${Math.round(toi * 1_000_000) / 1000} ms into this frame; the remaining interval uses the reflected velocity.`;
      return;
    }
  }

  body.x += body.velocity * dt;
  if (body.x < 55 && body.velocity < 0) {
    body.x = 55;
    body.velocity = Number(speed.value);
  }
}

function draw() {
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);

  context.globalAlpha = 0.18;
  context.fillRect(0, height - 68, width, 1);
  context.globalAlpha = 1;

  context.fillRect(wall.x - wall.halfWidth, 64, wall.halfWidth * 2, height - 132);

  context.save();
  context.translate(body.x, height / 2);
  context.fillRect(-body.halfWidth, -body.halfWidth, body.halfWidth * 2, body.halfWidth * 2);
  context.restore();

  const direction = Math.sign(body.velocity) || 1;
  const pathLength = Math.min(190, Math.abs(body.velocity) * 0.12);
  context.globalAlpha = 0.38;
  context.beginPath();
  context.moveTo(body.x, height / 2);
  context.lineTo(body.x + direction * pathLength, height / 2);
  context.stroke();
  context.globalAlpha = 1;

  if (lastImpact !== null) {
    context.beginPath();
    context.arc(wall.x - wall.halfWidth, height / 2, 10, 0, Math.PI * 2);
    context.stroke();
  }
}

function frame(timestamp) {
  if (previousTimestamp === null) previousTimestamp = timestamp;
  const dt = Math.min((timestamp - previousTimestamp) / 1000, 1 / 20);
  previousTimestamp = timestamp;
  step(dt);
  draw();
  requestAnimationFrame(frame);
}

speed.addEventListener("input", () => {
  updateLabels();
  reset();
});
restitution.addEventListener("input", updateLabels);
restart.addEventListener("click", reset);

updateLabels();
reset();
requestAnimationFrame(frame);
