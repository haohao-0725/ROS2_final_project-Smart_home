setInterval(() => {
  fetch("/api/maintenance")
    .then(res => res.json())
    .then(data => {

      const isMaintenance = (data.maintenance === true);

      if (isMaintenance) {
        // 進入維護
        if (window.location.pathname !== "/maintain") {
          window.location.href = "/maintain";
        }
      } else {
        // 非維護（包含 false / null / 沒資料）
        if (window.location.pathname === "/maintain") {
          window.location.href = "/";
        }
      }

    })
    .catch(err => console.error("maintenance check error:", err));
}, 1000);
