(() => {
  const workingLabels = [
    [/delete/i, "Deleting..."],
    [/remove/i, "Removing..."],
    [/hard reset|reset/i, "Resetting..."],
    [/pause/i, "Pausing..."],
    [/run|start/i, "Starting..."],
    [/upload/i, "Uploading..."],
    [/save/i, "Saving..."],
    [/sign in/i, "Signing in..."],
    [/sign out/i, "Signing out..."],
    [/next|queue/i, "Queuing..."],
    [/move/i, "Moving..."],
    [/rename/i, "Renaming..."]
  ];

  const submitterText = (submitter) => {
    if (!submitter) return "";
    return submitter.tagName === "INPUT" ? submitter.value : submitter.textContent.trim();
  };

  const setSubmitterText = (submitter, text) => {
    if (!submitter) return;

    if (submitter.tagName === "INPUT") {
      submitter.value = text;
    } else {
      submitter.textContent = text;
    }
  };

  const workingLabelFor = (submitter) => {
    const text = submitterText(submitter);
    const match = workingLabels.find(([pattern]) => pattern.test(text));
    return match ? match[1] : "Working...";
  };

  const showLoadingOverlay = (label) => {
    let overlay = document.querySelector("[data-loading-overlay]");

    if (!overlay) {
      overlay = document.createElement("div");
      overlay.className = "loading-overlay";
      overlay.dataset.loadingOverlay = "true";
      overlay.setAttribute("role", "status");
      overlay.setAttribute("aria-live", "polite");
      overlay.innerHTML = '<span class="loading-spinner" aria-hidden="true"></span><span data-loading-label></span>';
      document.body.appendChild(overlay);
    }

    overlay.querySelector("[data-loading-label]").textContent = label;
    overlay.hidden = false;
  };

  const submitAfterPaint = (form) => {
    requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        HTMLFormElement.prototype.submit.call(form);
      });
    });
  };

  const formatRemaining = (seconds) => {
    const clamped = Math.max(0, seconds);
    const minutes = Math.floor(clamped / 60).toString().padStart(2, "0");
    const remainder = Math.floor(clamped % 60).toString().padStart(2, "0");
    return `${minutes}:${remainder}`;
  };

  const updateCountdowns = () => {
    const playerRunning = document.querySelector("[data-player-running='true']");

    document.querySelectorAll("[data-countdown-started-at][data-countdown-duration]").forEach((timer) => {
      const startedAt = Date.parse(timer.dataset.countdownStartedAt);
      const duration = Number.parseInt(timer.dataset.countdownDuration, 10);
      if (Number.isNaN(startedAt) || Number.isNaN(duration) || duration <= 0) return;

      const elapsed = Math.floor((Date.now() - startedAt) / 1000);
      const remaining = Math.max(0, duration - elapsed);
      timer.textContent = `${timer.dataset.countdownPrefix} ${formatRemaining(remaining)}`;

      if (remaining === 0 && playerRunning && !window.__previewReloadQueued) {
        window.__previewReloadQueued = true;
        window.setTimeout(() => {
          refreshPreview();
          window.__previewReloadQueued = false;
        }, 500);
      }
    });
  };

  const setPreviewImage = (pane, image) => {
    const name = pane.querySelector("[data-preview-name]");
    const existingImage = pane.querySelector("[data-preview-image]");
    const existingEmpty = pane.querySelector("[data-preview-empty]");

    if (name) name.textContent = image ? image.name : "Waiting";

    if (!image) {
      existingImage?.remove();
      if (!existingEmpty) {
        const empty = document.createElement("div");
        empty.className = "empty-preview";
        empty.dataset.previewEmpty = "true";
        empty.textContent = "No image";
        pane.appendChild(empty);
      }
      return;
    }

    if (existingImage) {
      if (existingImage.getAttribute("src") !== image.thumbnail_url) {
        existingImage.setAttribute("src", image.thumbnail_url);
      }
      existingImage.setAttribute("alt", image.name);
      existingEmpty?.remove();
      return;
    }

    const img = document.createElement("img");
    img.dataset.previewImage = "true";
    img.src = image.thumbnail_url;
    img.alt = image.name;
    existingEmpty?.replaceWith(img);
  };

  const setPreviewTimer = (timer, label, timerState) => {
    if (!timer) return;

    if (!timerState) {
      delete timer.dataset.countdownStartedAt;
      delete timer.dataset.countdownDuration;
      timer.classList.add("pending");
      timer.textContent = "Timer pending";
      return;
    }

    timer.classList.remove("pending");
    timer.dataset.countdownPrefix = label;
    timer.dataset.countdownStartedAt = timerState.started_at;
    timer.dataset.countdownDuration = timerState.duration;
  };

  const refreshPreview = async () => {
    const grid = document.querySelector("[data-preview-url]");
    if (!grid || grid.dataset.previewLoading === "true") return;

    grid.dataset.previewLoading = "true";
    try {
      const response = await fetch(grid.dataset.previewUrl, {
        headers: { Accept: "application/json" },
        cache: "no-store"
      });
      if (!response.ok) return;

      const payload = await response.json();
      const currentPane = grid.querySelector("[data-preview-pane='current']");
      const nextPane = grid.querySelector("[data-preview-pane='next']");

      if (currentPane) {
        setPreviewImage(currentPane, payload.preview.current);
        setPreviewTimer(currentPane.querySelector(".preview-timer"), "Changes in", payload.preview.timer);
      }

      if (nextPane) {
        setPreviewImage(nextPane, payload.preview.next);
        setPreviewTimer(nextPane.querySelector(".preview-timer"), "Up in", payload.preview.timer);
      }

      const shell = document.querySelector("[data-player-running]");
      if (shell) shell.dataset.playerRunning = payload.running ? "true" : "false";
      updateCountdowns();
    } finally {
      delete grid.dataset.previewLoading;
    }
  };

  updateCountdowns();
  window.setInterval(updateCountdowns, 1000);
  window.setInterval(refreshPreview, 2000);

  document.addEventListener("submit", (event) => {
    const form = event.target;
    if (!(form instanceof HTMLFormElement) || form.dataset.noLoading === "true") return;
    if (form.dataset.loadingSubmitted === "true") return;

    const confirmation = form.dataset.turboConfirm;
    if (confirmation && !window.confirm(confirmation)) {
      event.preventDefault();
      return;
    }

    event.preventDefault();

    const submitter = event.submitter || form.querySelector("button[type='submit'], button:not([type]), input[type='submit']");
    const label = workingLabelFor(submitter);
    form.classList.add("is-submitting");
    form.setAttribute("aria-busy", "true");
    form.dataset.loadingSubmitted = "true";

    form.querySelectorAll("button[type='submit'], button:not([type]), input[type='submit']").forEach((button) => {
      button.disabled = true;
    });

    if (submitter) {
      submitter.classList.add("is-submitting");
      submitter.dataset.originalLabel = submitterText(submitter);
      setSubmitterText(submitter, label);
    }

    showLoadingOverlay(label);
    submitAfterPaint(form);
  }, true);
})();
