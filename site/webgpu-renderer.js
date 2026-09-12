const BYTES_PER_VERTEX = 9 * Float32Array.BYTES_PER_ELEMENT;
const CLEAR_COLOR = { r: 0.055, g: 0.085, b: 0.115, a: 1 };

const SHADER = /* wgsl */ `
struct Uniforms {
  aspect: f32,
  tan_half_fov: f32,
  near_plane: f32,
  far_plane: f32,
};

@group(0) @binding(0)
var<uniform> uniforms: Uniforms;

struct VertexInput {
  @location(0) position: vec3f,
  @location(1) normal: vec3f,
  @location(2) uv: vec2f,
  @location(3) material: f32,
};

struct VertexOutput {
  @builtin(position) position: vec4f,
  @location(0) normal: vec3f,
  @location(1) uv: vec2f,
  @location(2) @interpolate(flat) material: f32,
  @location(3) depth: f32,
};

@vertex
fn vertex_main(input: VertexInput) -> VertexOutput {
  let y_scale = 1.0 / uniforms.tan_half_fov;
  let x_scale = y_scale / uniforms.aspect;
  let z_scale = uniforms.far_plane / (uniforms.far_plane - uniforms.near_plane);
  let z_offset = (uniforms.far_plane * uniforms.near_plane) /
    (uniforms.far_plane - uniforms.near_plane);

  var output: VertexOutput;
  output.position = vec4f(
    input.position.x * x_scale,
    input.position.y * y_scale,
    z_scale * input.position.z - z_offset,
    input.position.z,
  );
  output.normal = input.normal;
  output.uv = input.uv;
  output.material = input.material;
  output.depth = input.position.z;
  return output;
}

fn grid_line(uv: vec2f) -> f32 {
  let cell = fract(uv);
  let edge = min(cell, vec2f(1.0) - cell);
  let aa = max(fwidth(uv) * 1.35, vec2f(0.002));
  let x_line = 1.0 - smoothstep(0.0, aa.x, edge.x);
  let y_line = 1.0 - smoothstep(0.0, aa.y, edge.y);
  return max(x_line, y_line);
}

fn box_border(uv: vec2f) -> f32 {
  let edge = min(uv, vec2f(1.0) - uv);
  let border = 1.0 - smoothstep(0.035, 0.07, min(edge.x, edge.y));
  let diagonal_a = 1.0 - smoothstep(0.025, 0.055, abs(uv.x - uv.y));
  let diagonal_b = 1.0 - smoothstep(0.025, 0.055, abs((1.0 - uv.x) - uv.y));
  return max(border, max(diagonal_a, diagonal_b) * 0.55);
}

fn hash(p: vec2f) -> f32 {
  return fract(sin(dot(p, vec2f(127.1, 311.7))) * 43758.5453);
}

@fragment
fn fragment_main(input: VertexOutput) -> @location(0) vec4f {
  var base: vec3f;
  var detail: vec3f;
  var pattern = 0.0;

  if input.material < 0.5 {
    base = vec3f(0.105, 0.13, 0.16);
    detail = vec3f(0.22, 0.27, 0.33);
    pattern = grid_line(input.uv) * 0.34;
  } else if input.material < 1.5 {
    base = vec3f(0.24, 0.28, 0.33);
    detail = vec3f(0.43, 0.49, 0.56);
    var brick_uv = input.uv;
    brick_uv.x += (floor(brick_uv.y) - 2.0 * floor(brick_uv.y / 2.0)) * 0.5;
    pattern = grid_line(brick_uv) * 0.48;
  } else if input.material < 2.5 {
    base = vec3f(0.34, 0.38, 0.43);
    detail = vec3f(0.46, 0.51, 0.57);
    let speckle = select(0.0, 1.0, hash(floor(input.uv * 5.0)) >= 0.91);
    pattern = speckle * 0.18;
  } else if input.material < 3.5 {
    base = vec3f(0.42, 0.25, 0.09);
    detail = vec3f(0.88, 0.58, 0.17);
    pattern = box_border(fract(input.uv));
  } else {
    base = vec3f(0.86, 0.19, 0.17);
    detail = vec3f(1.0, 0.52, 0.34);
    pattern = 0.28;
  }

  let light_direction = normalize(vec3f(-0.35, 0.82, 0.26));
  let diffuse = 0.62 + 0.38 * max(dot(normalize(input.normal), light_direction), 0.0);
  var color = mix(base, detail, clamp(pattern, 0.0, 1.0)) * diffuse;

  let fog_color = vec3f(0.055, 0.085, 0.115);
  let fog = smoothstep(540.0, 1100.0, input.depth);
  color = mix(color, fog_color, fog * 0.72);
  return vec4f(color, 1.0);
}
`;

function nextBufferSize(required) {
  let size = 4_096;
  while (size < required) size *= 2;
  return size;
}

export async function createWebGpuRenderer(canvas, options) {
  if (!navigator.gpu) return null;

  const adapter = await navigator.gpu.requestAdapter({ powerPreference: "high-performance" });
  if (!adapter) return null;
  const device = await adapter.requestDevice();

  const format = navigator.gpu.getPreferredCanvasFormat();
  const shader = device.createShaderModule({ label: "sandbox shader", code: SHADER });
  const compilation = await shader.getCompilationInfo();
  const shaderErrors = compilation.messages.filter((message) => message.type === "error");
  if (shaderErrors.length > 0) {
    throw new Error(`WebGPU shader compile failed: ${shaderErrors.map((error) => error.message).join("; ")}`);
  }

  const pipeline = device.createRenderPipeline({
    label: "sandbox pipeline",
    layout: "auto",
    vertex: {
      module: shader,
      entryPoint: "vertex_main",
      buffers: [
        {
          arrayStride: BYTES_PER_VERTEX,
          stepMode: "vertex",
          attributes: [
            { shaderLocation: 0, offset: 0, format: "float32x3" },
            { shaderLocation: 1, offset: 12, format: "float32x3" },
            { shaderLocation: 2, offset: 24, format: "float32x2" },
            { shaderLocation: 3, offset: 32, format: "float32" },
          ],
        },
      ],
    },
    fragment: {
      module: shader,
      entryPoint: "fragment_main",
      targets: [{ format }],
    },
    primitive: {
      topology: "triangle-list",
      cullMode: "none",
    },
    depthStencil: {
      format: "depth24plus",
      depthWriteEnabled: true,
      depthCompare: "less-equal",
    },
  });

  const uniformBuffer = device.createBuffer({
    label: "sandbox projection uniforms",
    size: 4 * Float32Array.BYTES_PER_ELEMENT,
    usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
  });
  const bindGroup = device.createBindGroup({
    label: "sandbox projection bind group",
    layout: pipeline.getBindGroupLayout(0),
    entries: [{ binding: 0, resource: { buffer: uniformBuffer } }],
  });

  // Delay acquiring a canvas context until the WebGPU device and shaders are known-good.
  // This preserves the caller's ability to fall back to WebGL2 if adapter/device setup fails.
  const context = canvas.getContext("webgpu");
  if (!context) return null;
  context.configure({ device, format, alphaMode: "opaque" });

  let vertexCapacity = 4_096;
  let vertexBuffer = device.createBuffer({
    label: "sandbox dynamic vertices",
    size: vertexCapacity,
    usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
  });
  let depthTexture = null;
  let depthView = null;
  let width = 0;
  let height = 0;
  let lost = false;

  device.lost.then((info) => {
    lost = true;
    console.error(`WebGPU device lost: ${info.message}`);
  });

  function resize(nextWidth, nextHeight) {
    if (nextWidth === width && nextHeight === height && depthView) return;
    width = Math.max(1, nextWidth);
    height = Math.max(1, nextHeight);
    depthTexture?.destroy();
    depthTexture = device.createTexture({
      label: "sandbox depth",
      size: [width, height],
      format: "depth24plus",
      usage: GPUTextureUsage.RENDER_ATTACHMENT,
    });
    depthView = depthTexture.createView();
  }

  function render(vertices) {
    if (lost) throw new Error("WebGPU device was lost");
    const byteLength = vertices.byteLength;
    if (byteLength > vertexCapacity) {
      vertexBuffer.destroy();
      vertexCapacity = nextBufferSize(byteLength);
      vertexBuffer = device.createBuffer({
        label: "sandbox dynamic vertices",
        size: vertexCapacity,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
      });
    }
    if (byteLength > 0) device.queue.writeBuffer(vertexBuffer, 0, vertices);

    device.queue.writeBuffer(
      uniformBuffer,
      0,
      new Float32Array([
        canvas.width / canvas.height,
        Math.tan(options.fovRadians / 2),
        options.nearPlane,
        options.farPlane,
      ]),
    );

    const encoder = device.createCommandEncoder({ label: "sandbox frame" });
    const pass = encoder.beginRenderPass({
      label: "sandbox render pass",
      colorAttachments: [
        {
          view: context.getCurrentTexture().createView(),
          clearValue: CLEAR_COLOR,
          loadOp: "clear",
          storeOp: "store",
        },
      ],
      depthStencilAttachment: {
        view: depthView,
        depthClearValue: 1,
        depthLoadOp: "clear",
        depthStoreOp: "store",
      },
    });
    pass.setPipeline(pipeline);
    pass.setBindGroup(0, bindGroup);
    pass.setVertexBuffer(0, vertexBuffer);
    pass.draw(vertices.length / 9);
    pass.end();
    device.queue.submit([encoder.finish()]);
  }

  return {
    backend: "WebGPU",
    resize,
    render,
  };
}
