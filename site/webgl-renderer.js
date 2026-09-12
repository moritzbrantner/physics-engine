const FLOATS_PER_VERTEX = 9;

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const message = gl.getShaderInfoLog(shader);
    gl.deleteShader(shader);
    throw new Error(`WebGL shader compile failed: ${message}`);
  }
  return shader;
}

export function createWebGlRenderer(canvas, options) {
  const gl = canvas.getContext("webgl2", {
    antialias: true,
    alpha: false,
    depth: true,
    premultipliedAlpha: false,
  });
  if (!gl) return null;

  const vertexShader = compileShader(
    gl,
    gl.VERTEX_SHADER,
    `#version 300 es
    precision highp float;
    layout(location = 0) in vec3 aPosition;
    layout(location = 1) in vec3 aNormal;
    layout(location = 2) in vec2 aUv;
    layout(location = 3) in float aMaterial;

    uniform float uAspect;
    uniform float uTanHalfFov;
    uniform float uNear;
    uniform float uFar;

    out vec3 vNormal;
    out vec2 vUv;
    flat out float vMaterial;
    out float vDepth;

    void main() {
      float yScale = 1.0 / uTanHalfFov;
      float xScale = yScale / uAspect;
      float zScale = (uFar + uNear) / (uFar - uNear);
      float zOffset = (2.0 * uFar * uNear) / (uFar - uNear);
      gl_Position = vec4(
        aPosition.x * xScale,
        aPosition.y * yScale,
        zScale * aPosition.z - zOffset,
        aPosition.z
      );
      vNormal = aNormal;
      vUv = aUv;
      vMaterial = aMaterial;
      vDepth = aPosition.z;
    }`,
  );

  const fragmentShader = compileShader(
    gl,
    gl.FRAGMENT_SHADER,
    `#version 300 es
    precision highp float;

    in vec3 vNormal;
    in vec2 vUv;
    flat in float vMaterial;
    in float vDepth;
    out vec4 outColor;

    float gridLine(vec2 uv) {
      vec2 cell = fract(uv);
      vec2 edge = min(cell, 1.0 - cell);
      vec2 aa = max(fwidth(uv) * 1.35, vec2(0.002));
      float xLine = 1.0 - smoothstep(0.0, aa.x, edge.x);
      float yLine = 1.0 - smoothstep(0.0, aa.y, edge.y);
      return max(xLine, yLine);
    }

    float boxBorder(vec2 uv) {
      vec2 edge = min(uv, 1.0 - uv);
      float border = 1.0 - smoothstep(0.035, 0.07, min(edge.x, edge.y));
      float diagonalA = 1.0 - smoothstep(0.025, 0.055, abs(uv.x - uv.y));
      float diagonalB = 1.0 - smoothstep(0.025, 0.055, abs((1.0 - uv.x) - uv.y));
      return max(border, max(diagonalA, diagonalB) * 0.55);
    }

    float hash(vec2 p) {
      return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
    }

    void main() {
      vec3 base;
      vec3 detail;
      float pattern = 0.0;

      if (vMaterial < 0.5) {
        base = vec3(0.105, 0.13, 0.16);
        detail = vec3(0.22, 0.27, 0.33);
        pattern = gridLine(vUv) * 0.34;
      } else if (vMaterial < 1.5) {
        base = vec3(0.24, 0.28, 0.33);
        detail = vec3(0.43, 0.49, 0.56);
        vec2 brickUv = vUv;
        brickUv.x += mod(floor(brickUv.y), 2.0) * 0.5;
        pattern = gridLine(brickUv) * 0.48;
      } else if (vMaterial < 2.5) {
        base = vec3(0.34, 0.38, 0.43);
        detail = vec3(0.46, 0.51, 0.57);
        float speckle = step(0.91, hash(floor(vUv * 5.0)));
        pattern = speckle * 0.18;
      } else if (vMaterial < 3.5) {
        base = vec3(0.42, 0.25, 0.09);
        detail = vec3(0.88, 0.58, 0.17);
        pattern = boxBorder(fract(vUv));
      } else {
        base = vec3(0.86, 0.19, 0.17);
        detail = vec3(1.0, 0.52, 0.34);
        pattern = 0.28;
      }

      vec3 lightDirection = normalize(vec3(-0.35, 0.82, 0.26));
      float diffuse = 0.62 + 0.38 * max(dot(normalize(vNormal), lightDirection), 0.0);
      vec3 color = mix(base, detail, clamp(pattern, 0.0, 1.0)) * diffuse;

      vec3 fogColor = vec3(0.055, 0.085, 0.115);
      float fog = smoothstep(540.0, 1100.0, vDepth);
      color = mix(color, fogColor, fog * 0.72);

      outColor = vec4(color, 1.0);
    }`,
  );

  const program = gl.createProgram();
  gl.attachShader(program, vertexShader);
  gl.attachShader(program, fragmentShader);
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const message = gl.getProgramInfoLog(program);
    gl.deleteProgram(program);
    throw new Error(`WebGL program link failed: ${message}`);
  }
  gl.deleteShader(vertexShader);
  gl.deleteShader(fragmentShader);

  const buffer = gl.createBuffer();
  const vao = gl.createVertexArray();
  gl.bindVertexArray(vao);
  gl.bindBuffer(gl.ARRAY_BUFFER, buffer);

  const stride = FLOATS_PER_VERTEX * Float32Array.BYTES_PER_ELEMENT;
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, stride, 0);
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 3, gl.FLOAT, false, stride, 3 * Float32Array.BYTES_PER_ELEMENT);
  gl.enableVertexAttribArray(2);
  gl.vertexAttribPointer(2, 2, gl.FLOAT, false, stride, 6 * Float32Array.BYTES_PER_ELEMENT);
  gl.enableVertexAttribArray(3);
  gl.vertexAttribPointer(3, 1, gl.FLOAT, false, stride, 8 * Float32Array.BYTES_PER_ELEMENT);

  gl.enable(gl.DEPTH_TEST);
  gl.depthFunc(gl.LEQUAL);
  gl.disable(gl.BLEND);
  gl.disable(gl.CULL_FACE);

  const uniforms = {
    aspect: gl.getUniformLocation(program, "uAspect"),
    tanHalfFov: gl.getUniformLocation(program, "uTanHalfFov"),
    near: gl.getUniformLocation(program, "uNear"),
    far: gl.getUniformLocation(program, "uFar"),
  };

  function resize(width, height) {
    gl.viewport(0, 0, width, height);
  }

  function render(vertices) {
    gl.clearColor(0.055, 0.085, 0.115, 1);
    gl.clearDepth(1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    gl.useProgram(program);
    gl.bindVertexArray(vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.DYNAMIC_DRAW);

    gl.uniform1f(uniforms.aspect, canvas.width / canvas.height);
    gl.uniform1f(uniforms.tanHalfFov, Math.tan(options.fovRadians / 2));
    gl.uniform1f(uniforms.near, options.nearPlane);
    gl.uniform1f(uniforms.far, options.farPlane);
    gl.drawArrays(gl.TRIANGLES, 0, vertices.length / FLOATS_PER_VERTEX);
  }

  return {
    backend: "WebGL2",
    resize,
    render,
  };
}
