/* global mermaid, document$ */
(function () {
  function applyMermaid() {
    if (typeof mermaid === "undefined") {
      return;
    }

    const codeBlocks = document.querySelectorAll("pre code.language-mermaid");
    codeBlocks.forEach((codeBlock) => {
      const pre = codeBlock.parentElement;
      if (!pre || pre.dataset.mermaidConverted === "true") {
        return;
      }

      const wrapper = document.createElement("div");
      wrapper.className = "mermaid";
      wrapper.textContent = codeBlock.textContent;

      pre.replaceWith(wrapper);
    });

    mermaid.initialize({
      startOnLoad: false,
      securityLevel: "loose",
      theme: document.body.getAttribute("data-md-color-scheme") === "slate" ? "dark" : "default",
    });

    mermaid.run({
      querySelector: ".mermaid",
    });
  }

  if (typeof document$ !== "undefined") {
    document$.subscribe(applyMermaid);
  } else {
    document.addEventListener("DOMContentLoaded", applyMermaid);
  }
})();
