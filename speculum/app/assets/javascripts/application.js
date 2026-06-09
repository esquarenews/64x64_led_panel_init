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
    document.querySelectorAll("[data-countdown-started-at][data-countdown-duration]").forEach((timer) => {
      const startedAt = Date.parse(timer.dataset.countdownStartedAt);
      const duration = Number.parseInt(timer.dataset.countdownDuration, 10);
      if (Number.isNaN(startedAt) || Number.isNaN(duration) || duration <= 0) return;

      const elapsed = Math.floor((Date.now() - startedAt) / 1000);
      const remaining = Math.max(0, duration - elapsed);
      timer.textContent = `${timer.dataset.countdownPrefix} ${formatRemaining(remaining)}`;

      if (remaining === 0 && elapsed <= duration + 2 && !window.__previewReloadQueued) {
        window.__previewReloadQueued = true;
        window.setTimeout(() => window.location.reload(), 1500);
      }
    });
  };

  updateCountdowns();
  window.setInterval(updateCountdowns, 1000);

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
