function changePattern() {
  var pattern_id = document.getElementById("pattern_select").value;
  $.get("/api/set_pattern?pattern=" + pattern_id)
}
