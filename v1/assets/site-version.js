// Line switcher for the two published documentation sites.
// The site root redirects to /v2/. This control is added to the Material header.
(function () {
  var versions = ["v2", "v1"];
  var fallback = "v1";
  var siteRoot = "/flexiv_tdk/";

  function splitPath(pathname) {
    var at = pathname.indexOf(siteRoot);
    if (at === -1) return null;
    var after = pathname.slice(at + siteRoot.length);
    var match = after.match(/^(v1|v2)(\/.*)?$/);
    if (!match) return null;
    return { version: match[1], rest: match[2] || "/" };
  }

  var loaded = splitPath(location.pathname);
  var current = loaded ? loaded.version : fallback;

  function destination(version, parsed) {
    var rest = parsed ? parsed.rest : "/";
    if (version === "v1") {
      rest = rest.replace(/^\/(?:zh-Hant|zh|de|ja)(?=\/|$)/, "");
      if (!rest) rest = "/";
    }
    if (rest.charAt(0) !== "/") rest = "/" + rest;
    return siteRoot + version + rest;
  }

  function go(version) {
    // Instant navigation changes the page without reloading this script.
    var parsed = splitPath(location.pathname);
    var next = destination(version, parsed);
    if (!parsed) {
      location.href = "https://flexivrobotics.github.io" + next;
      return;
    }
    if (version === current) return;
    fetch(next, { method: "HEAD" })
      .then(function (response) {
        location.href = response.ok ? next : siteRoot + version + "/";
      })
      .catch(function () {
        location.href = next;
      });
  }

  var header = document.querySelector(".md-header__inner");
  if (!header) return;

  var label = document.createElement("label");
  label.className = "tdk-site-version";
  var caption = document.createElement("span");
  caption.className = "tdk-site-version__caption";
  caption.textContent = "Version";
  var select = document.createElement("select");
  select.id = "tdk-site-version-select";
  select.setAttribute("aria-label", "Documentation version");
  versions.forEach(function (id) {
    var option = document.createElement("option");
    option.value = id;
    option.textContent = id === "v2" ? "v2 (default)" : id;
    if (id === current) option.selected = true;
    select.appendChild(option);
  });
  select.addEventListener("change", function () {
    go(select.value);
  });
  label.appendChild(caption);
  label.appendChild(select);
  header.appendChild(label);
})();
