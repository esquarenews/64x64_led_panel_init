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

  document.addEventListener("submit", (event) => {
    const form = event.target;
    if (!(form instanceof HTMLFormElement) || form.dataset.noLoading === "true") return;

    const confirmation = form.dataset.turboConfirm;
    if (confirmation && !window.confirm(confirmation)) {
      event.preventDefault();
      return;
    }

    const submitter = event.submitter || form.querySelector("button[type='submit'], button:not([type]), input[type='submit']");
    form.classList.add("is-submitting");
    form.setAttribute("aria-busy", "true");

    form.querySelectorAll("button[type='submit'], button:not([type]), input[type='submit']").forEach((button) => {
      button.disabled = true;
    });

    if (submitter) {
      submitter.classList.add("is-submitting");
      submitter.dataset.originalLabel = submitterText(submitter);
      setSubmitterText(submitter, workingLabelFor(submitter));
    }
  }, true);
})();
