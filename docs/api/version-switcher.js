// API 版本下拉切换器：从 <select data-api-base> 指向的目录读取 versions.json
// （由 build_api_docs.sh 生成），切换时跳转到 <base>doxygen/<tag>/index.html。
// data-api-base：英文页为 "./"，其他语言页因静态文件只发布在根语言目录而为 "../../api/"。
(function () {
  var select = document.getElementById("api-version-select");
  if (!select) return;

  var base = select.getAttribute("data-api-base") || "./";
  var fallback = { versions: ["v2.1", "v1.6.1"], latest: "v2.1" };

  function populate(data) {
    data.versions.forEach(function (v) {
      var opt = document.createElement("option");
      opt.value = v;
      opt.textContent = v === data.latest ? v + " (latest)" : v;
      select.appendChild(opt);
    });
    select.value = data.latest;
  }

  fetch(base + "versions.json")
    .then(function (r) {
      return r.ok ? r.json() : Promise.reject(new Error(r.status));
    })
    .then(populate)
    .catch(function () {
      populate(fallback);
    });

  select.addEventListener("change", function () {
    window.location.href = base + "doxygen/" + select.value + "/index.html";
  });
})();
