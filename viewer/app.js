const canvas = document.getElementById("mapCanvas");
    const ctx = canvas.getContext("2d");
    const hoverSlot = document.getElementById("hoverSlot");
    const planStatus = document.getElementById("planStatus");
    const spaceCount = document.getElementById("spaceCount");
    const pathCount = document.getElementById("pathCount");
    const trajectoryCount = document.getElementById("trajectoryCount");
    const progressStatus = document.getElementById("progressStatus");
    const gearStatus = document.getElementById("gearStatus");
    const speedStatus = document.getElementById("speedStatus");
    const steerStatus = document.getElementById("steerStatus");
    const accelStatus = document.getElementById("accelStatus");
    const yawStatus = document.getElementById("yawStatus");
    const curvatureStatus = document.getElementById("curvatureStatus");
    const distanceStatus = document.getElementById("distanceStatus");
    const timeStatus = document.getElementById("timeStatus");
    const startDriveButton = document.getElementById("startDrive");
    const pauseDriveButton = document.getElementById("pauseDrive");
    const resetDriveButton = document.getElementById("resetDrive");
    const resetViewButton = document.getElementById("resetView");
    const toast = document.getElementById("toast");

    const state = {
      map: null,
      parkingSpaces: [],
      selected: null,
      hovered: null,
      path: [],
      segments: [],
      trajectory: [],
      vehicle: null,
      animationIndex: 0,
      animationTimer: null,
      scale: 1,
      offsetX: 0,
      offsetY: 0,
      dragging: false,
      dragStart: null,
      lastMouse: null,
      devicePixelRatio: window.devicePixelRatio || 1,
    };

    const colors = {
      road: "#474a45",
      roadStroke: "#5b5f58",
      space: "#d7d2c5",
      target: "#52b7d8",
      charging: "#90d9bd",
      accessible: "#9dc8f0",
      reserved: "#d9bb52",
      vehicle: "#777b73",
      barrier: "#4c4d49",
      gate: "#3583a3",
      ramp: "#b9b3a4",
      walkway: "#bdb8a4",
      selectedStroke: "#ffffff",
      hoverStroke: "#f1c75d",
      globalPath: "#f1c75d",
      reversePath: "#de735e",
      localPath: "#6bd1ff",
      trace: "#f0f0ea",
      ego: "#f0f0ea",
      egoNose: "#52b7d8",
    };

    function resizeCanvas() {
      const rect = canvas.getBoundingClientRect();
      state.devicePixelRatio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(rect.width * state.devicePixelRatio));
      canvas.height = Math.max(1, Math.floor(rect.height * state.devicePixelRatio));
      ctx.setTransform(state.devicePixelRatio, 0, 0, state.devicePixelRatio, 0, 0);
      draw();
    }

    function fitView() {
      if (!state.map) return;
      const rect = canvas.getBoundingClientRect();
      const bounds = state.map.bounds;
      const width = bounds.maxX - bounds.minX;
      const height = bounds.maxY - bounds.minY;
      const padding = 34;
      const scaleX = (rect.width - padding * 2) / width;
      const scaleY = (rect.height - padding * 2) / height;
      state.scale = Math.max(1, Math.min(scaleX, scaleY));
      state.offsetX = padding - bounds.minX * state.scale + (rect.width - padding * 2 - width * state.scale) / 2;
      state.offsetY = padding + bounds.maxY * state.scale + (rect.height - padding * 2 - height * state.scale) / 2;
      draw();
    }

    function worldToScreen(point) {
      return {
        x: point.x * state.scale + state.offsetX,
        y: -point.y * state.scale + state.offsetY,
      };
    }

    function screenToWorld(point) {
      return {
        x: (point.x - state.offsetX) / state.scale,
        y: -(point.y - state.offsetY) / state.scale,
      };
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);
      if (!state.map) return;
      drawRoads();
      drawObjects();
      drawGlobalPath();
      drawLocalTrajectory();
      drawDrivenTrace();
      drawEgo();
    }

    function drawRoads() {
      for (const road of state.map.roads) {
        drawPolygon(road.polygon, colors.road, colors.roadStroke, 1);
      }
    }

    function drawObjects() {
      const objects = [...state.map.objects].sort((a, b) => objectLayer(a) - objectLayer(b));
      for (const object of objects) {
        const style = objectStyle(object);
        drawPolygon(object.polygon, style.fill, style.stroke, style.width);
      }
    }

    function objectLayer(object) {
      if (object.type === "parkingSpace") return 1;
      if (object.type === "pedestrianWalkway" || object.type === "ramp" || object.type === "gate") return 2;
      return 3;
    }

    function objectStyle(object) {
      if (object.type === "parkingSpace") {
        let fill = colors.space;
        if (object.subtype === "target") fill = colors.target;
        if (object.subtype === "charging") fill = colors.charging;
        if (object.subtype === "accessible") fill = colors.accessible;
        if (object.subtype === "reserved") fill = colors.reserved;
        if (state.selected && object.name === state.selected.name) fill = colors.target;
        const stroke = state.hovered && object.name === state.hovered.name
          ? colors.hoverStroke
          : state.selected && object.name === state.selected.name
            ? colors.selectedStroke
            : "#74766f";
        return { fill, stroke, width: state.selected && object.name === state.selected.name ? 2.2 : 1 };
      }
      if (object.type === "vehicle") return { fill: colors.vehicle, stroke: "#5a5d56", width: 1 };
      if (object.type === "barrier") return { fill: colors.barrier, stroke: "#343631", width: 1 };
      if (object.type === "gate") return { fill: colors.gate, stroke: "#27627a", width: 1 };
      if (object.type === "ramp") return { fill: colors.ramp, stroke: "#8f897b", width: 1 };
      if (object.type === "pedestrianWalkway") return { fill: colors.walkway, stroke: "#918b7a", width: 1 };
      return { fill: object.fill || "#999", stroke: "#555", width: 1 };
    }

    function drawPolygon(points, fill, stroke, lineWidth) {
      if (!points.length) return;
      ctx.beginPath();
      const first = worldToScreen(points[0]);
      ctx.moveTo(first.x, first.y);
      for (let i = 1; i < points.length; i += 1) {
        const point = worldToScreen(points[i]);
        ctx.lineTo(point.x, point.y);
      }
      ctx.closePath();
      ctx.fillStyle = fill;
      ctx.fill();
      ctx.strokeStyle = stroke;
      ctx.lineWidth = lineWidth;
      ctx.stroke();
    }

    function drawGlobalPath() {
      if (state.segments.length) {
        for (const segment of state.segments) {
          drawPath(
            segment.points,
            segment.gear === "reverse" ? colors.reversePath : colors.globalPath,
            segment.gear === "reverse",
            4,
          );
        }
        return;
      }
      drawPath(state.path, colors.globalPath, false, 4);
    }

    function drawLocalTrajectory() {
      if (state.trajectory.length < 2) return;
      const start = state.animationIndex;
      const end = Math.min(state.trajectory.length, start + 28);
      drawPath(state.trajectory.slice(start, end), colors.localPath, false, 3);
    }

    function drawDrivenTrace() {
      if (state.trajectory.length < 2) return;
      drawPath(state.trajectory.slice(0, state.animationIndex + 1), colors.trace, false, 2);
    }

    function drawPath(points, color, dashed, lineWidth) {
      if (points.length < 2) return;
      ctx.beginPath();
      const first = worldToScreen(points[0]);
      ctx.moveTo(first.x, first.y);
      for (let i = 1; i < points.length; i += 1) {
        const point = worldToScreen(points[i]);
        ctx.lineTo(point.x, point.y);
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = lineWidth;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.setLineDash(dashed ? [8, 6] : []);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    function drawEgo() {
      const vehicle = state.vehicle || state.map.scenario.egoStart;
      const point = worldToScreen(vehicle);
      ctx.save();
      ctx.translate(point.x, point.y);
      ctx.rotate(-vehicle.yaw);
      const length = Math.max(18, 4.6 * state.scale);
      const width = Math.max(9, 2.0 * state.scale);
      ctx.beginPath();
      ctx.rect(-length * 0.5, -width * 0.5, length, width);
      ctx.fillStyle = colors.ego;
      ctx.strokeStyle = "#10110f";
      ctx.lineWidth = 1.6;
      ctx.fill();
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(length * 0.5, 0);
      ctx.lineTo(length * 0.22, -width * 0.28);
      ctx.lineTo(length * 0.22, width * 0.28);
      ctx.closePath();
      ctx.fillStyle = colors.egoNose;
      ctx.fill();
      ctx.restore();
    }

    function canvasPoint(event) {
      const rect = canvas.getBoundingClientRect();
      return { x: event.clientX - rect.left, y: event.clientY - rect.top };
    }

    function pointInPolygon(point, polygon) {
      let inside = false;
      for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i, i += 1) {
        const pi = polygon[i];
        const pj = polygon[j];
        const intersects = ((pi.y > point.y) !== (pj.y > point.y))
          && point.x < ((pj.x - pi.x) * (point.y - pi.y)) / (pj.y - pi.y) + pi.x;
        if (intersects) inside = !inside;
      }
      return inside;
    }

    function findParkingSpace(worldPoint) {
      for (let i = state.parkingSpaces.length - 1; i >= 0; i -= 1) {
        const space = state.parkingSpaces[i];
        if (pointInPolygon(worldPoint, space.polygon)) return space;
      }
      return null;
    }

    async function planTo(space) {
      stopAnimation();
      state.selected = space;
      state.path = [];
      state.segments = [];
      state.trajectory = [];
      state.vehicle = null;
      state.animationIndex = 0;
      pathCount.textContent = "-";
      trajectoryCount.textContent = "-";
      progressStatus.textContent = "-";
      resetMicroStatus();
      startDriveButton.disabled = true;
      pauseDriveButton.disabled = true;
      resetDriveButton.disabled = true;
      planStatus.textContent = "规划中...";
      planStatus.classList.remove("error");
      draw();

      try {
        const payload = await postJson("/api/plan", {
          target: space.name,
          start: state.map.scenario.egoStart,
        });
        state.path = payload.path;
        state.segments = payload.segments || [];
        pathCount.textContent = String(payload.path.length);
        planStatus.textContent = "全局路径已生成";
        startDriveButton.disabled = false;
      } catch (error) {
        planStatus.textContent = error.message;
        planStatus.classList.add("error");
      } finally {
        draw();
      }
    }

    async function loadDriveTrajectory() {
      if (!state.selected) return;
      planStatus.textContent = "生成局部轨迹中...";
      startDriveButton.disabled = true;
      try {
        const payload = await postJson("/api/drive", {
          target: state.selected.name,
          start: state.map.scenario.egoStart,
        });
        state.trajectory = payload.points;
        state.animationIndex = 0;
        state.vehicle = state.trajectory[0] || null;
        trajectoryCount.textContent = String(state.trajectory.length);
        resetDriveButton.disabled = false;
        updateMicroStatus();
        planStatus.textContent = "局部轨迹已生成，开始行驶";
      } catch (error) {
        planStatus.textContent = error.message;
        planStatus.classList.add("error");
      }
    }

    async function startDrive() {
      if (!state.selected) return;
      if (!state.trajectory.length) await loadDriveTrajectory();
      if (!state.trajectory.length) return;
      startDriveButton.disabled = true;
      pauseDriveButton.disabled = false;
      animate();
    }

    function animate() {
      stopAnimation();
      state.animationTimer = window.setInterval(() => {
        if (state.animationIndex >= state.trajectory.length - 1) {
          stopAnimation();
          startDriveButton.disabled = false;
          pauseDriveButton.disabled = true;
          planStatus.textContent = "行驶完成";
          return;
        }
        state.animationIndex += 1;
        state.vehicle = state.trajectory[state.animationIndex];
        updateMicroStatus();
        draw();
      }, 45);
    }

    function stopAnimation() {
      if (state.animationTimer !== null) {
        window.clearInterval(state.animationTimer);
        state.animationTimer = null;
      }
    }

    function resetDrive() {
      stopAnimation();
      state.animationIndex = 0;
      state.vehicle = state.trajectory[0] || null;
      startDriveButton.disabled = !state.selected;
      pauseDriveButton.disabled = true;
      planStatus.textContent = state.selected ? "车辆已重置" : "点击车位生成全局路径";
      updateMicroStatus();
      draw();
    }

    function updateMicroStatus() {
      const vehicle = state.vehicle;
      if (!vehicle) {
        resetMicroStatus();
        return;
      }
      const index = state.animationIndex + 1;
      const total = Math.max(state.trajectory.length, 1);
      progressStatus.textContent = `${index}/${total}`;
      gearStatus.textContent = vehicle.gear === "reverse" ? "倒车" : "前进";
      speedStatus.textContent = `${vehicle.v.toFixed(2)} m/s`;
      steerStatus.textContent = `${toDeg(vehicle.steer || 0).toFixed(1)} deg`;
      accelStatus.textContent = `${(vehicle.acceleration || 0).toFixed(2)} m/s²`;
      yawStatus.textContent = `${toDeg(vehicle.yaw || 0).toFixed(1)} deg`;
      curvatureStatus.textContent = `${(vehicle.curvature || 0).toFixed(3)} 1/m`;
      distanceStatus.textContent = `${(vehicle.s || 0).toFixed(1)} m`;
      timeStatus.textContent = `${(vehicle.t || 0).toFixed(1)} s`;
    }

    function resetMicroStatus() {
      progressStatus.textContent = "-";
      gearStatus.textContent = "-";
      speedStatus.textContent = "-";
      steerStatus.textContent = "-";
      accelStatus.textContent = "-";
      yawStatus.textContent = "-";
      curvatureStatus.textContent = "-";
      distanceStatus.textContent = "-";
      timeStatus.textContent = "-";
    }

    function toDeg(radians) {
      return radians * 180 / Math.PI;
    }

    async function postJson(url, body) {
      const response = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const payload = await response.json();
      if (!response.ok) throw new Error(payload.error || "请求失败");
      return payload;
    }

    function updateHover(event) {
      const worldPoint = screenToWorld(canvasPoint(event));
      state.hovered = findParkingSpace(worldPoint);
      hoverSlot.textContent = state.hovered ? state.hovered.name : "未选择";
      canvas.style.cursor = state.hovered ? "pointer" : state.dragging ? "grabbing" : "crosshair";
      draw();
    }

    canvas.addEventListener("mousemove", (event) => {
      if (state.dragging && state.dragStart) {
        const point = canvasPoint(event);
        state.offsetX = state.dragStart.offsetX + point.x - state.dragStart.x;
        state.offsetY = state.dragStart.offsetY + point.y - state.dragStart.y;
        draw();
        return;
      }
      updateHover(event);
    });

    canvas.addEventListener("mousedown", (event) => {
      state.lastMouse = canvasPoint(event);
      state.dragging = true;
      state.dragStart = { ...state.lastMouse, offsetX: state.offsetX, offsetY: state.offsetY };
      canvas.style.cursor = "grabbing";
    });

    window.addEventListener("mouseup", () => {
      state.dragging = false;
      state.dragStart = null;
      canvas.style.cursor = state.hovered ? "pointer" : "crosshair";
    });

    canvas.addEventListener("click", (event) => {
      const point = canvasPoint(event);
      if (state.lastMouse && Math.hypot(point.x - state.lastMouse.x, point.y - state.lastMouse.y) > 4) return;
      const space = findParkingSpace(screenToWorld(point));
      if (space) planTo(space);
    });

    canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      const point = canvasPoint(event);
      const before = screenToWorld(point);
      const factor = event.deltaY < 0 ? 1.12 : 0.89;
      state.scale = Math.max(2, Math.min(80, state.scale * factor));
      const after = worldToScreen(before);
      state.offsetX += point.x - after.x;
      state.offsetY += point.y - after.y;
      draw();
    }, { passive: false });

    startDriveButton.addEventListener("click", startDrive);
    pauseDriveButton.addEventListener("click", () => {
      stopAnimation();
      pauseDriveButton.disabled = true;
      startDriveButton.disabled = false;
      planStatus.textContent = "已暂停";
    });
    resetDriveButton.addEventListener("click", resetDrive);
    resetViewButton.addEventListener("click", fitView);
    window.addEventListener("resize", () => {
      resizeCanvas();
      fitView();
    });

    async function loadMap() {
      const response = await fetch("/api/map");
      state.map = await response.json();
      state.parkingSpaces = state.map.objects.filter((object) => object.type === "parkingSpace");
      spaceCount.textContent = String(state.parkingSpaces.length);
      resizeCanvas();
      fitView();
    }

    loadMap().catch((error) => {
      toast.textContent = error.message;
      planStatus.textContent = "地图加载失败";
      planStatus.classList.add("error");
    });
