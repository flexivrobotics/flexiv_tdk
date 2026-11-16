// 首页轮播：自动播放（5s）、箭头/圆点导航、悬停暂停、标签页隐藏时暂停。
// 无 JS 时 viewport 退化为可横向滚动容器，图片仍可浏览。
(function () {
  var INTERVAL = 5000;

  document.querySelectorAll(".tdk-carousel").forEach(function (carousel) {
    var viewport = carousel.querySelector(".tdk-carousel-viewport");
    if (!viewport) return;

    // 用图片 alt 文本渲染行业标签（alt 已在各语言首页中本地化）
    Array.prototype.forEach.call(viewport.children, function (slide) {
      var img = slide.querySelector ? slide.querySelector("img") : null;
      if (img && img.alt) {
        var cap = document.createElement("span");
        cap.className = "tdk-carousel-caption";
        cap.textContent = img.alt;
        slide.appendChild(cap);
      }
    });

    var count = viewport.children.length;
    var prev = carousel.querySelector(".tdk-carousel-prev");
    var next = carousel.querySelector(".tdk-carousel-next");
    var dotsBox = carousel.querySelector(".tdk-carousel-dots");

    if (count <= 1) {
      [prev, next, dotsBox].forEach(function (el) {
        if (el) el.style.display = "none";
      });
      return;
    }

    var index = 0;
    var timer = null;
    var dots = [];

    for (var i = 0; i < count; i++) {
      (function (i) {
        var dot = document.createElement("button");
        dot.className = "tdk-carousel-dot";
        dot.setAttribute("aria-label", "Go to image " + (i + 1));
        dot.addEventListener("click", function () {
          goTo(i);
          restart();
        });
        dotsBox.appendChild(dot);
        dots.push(dot);
      })(i);
    }

    function updateDots() {
      dots.forEach(function (d, i) {
        d.classList.toggle("active", i === index);
      });
    }

    function goTo(i) {
      index = (i + count) % count;
      viewport.scrollTo({ left: viewport.clientWidth * index, behavior: "smooth" });
      updateDots();
    }

    function start() {
      if (timer === null) timer = setInterval(function () { goTo(index + 1); }, INTERVAL);
    }

    function stop() {
      clearInterval(timer);
      timer = null;
    }

    function restart() {
      stop();
      start();
    }

    prev.addEventListener("click", function () { goTo(index - 1); restart(); });
    next.addEventListener("click", function () { goTo(index + 1); restart(); });

    // 用户手动滑动时同步圆点状态
    viewport.addEventListener("scroll", function () {
      var i = Math.round(viewport.scrollLeft / viewport.clientWidth);
      if (i !== index) {
        index = i;
        updateDots();
      }
    }, { passive: true });

    carousel.addEventListener("mouseenter", stop);
    carousel.addEventListener("mouseleave", start);
    document.addEventListener("visibilitychange", function () {
      if (document.hidden) stop(); else start();
    });

    updateDots();
    start();
  });
})();
