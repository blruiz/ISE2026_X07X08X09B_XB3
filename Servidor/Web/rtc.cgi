t <html><head><title>RTC Control</title>
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
t var formUpdate = new periodicObj("rtc.cgx", 1000);
t function updateRTC() {
t  time = document.getElementById("current_time").value;
t  date = document.getElementById("current_date").value;
t  document.getElementById("current_time").value = time;
t  document.getElementById("current_date").value = date;
t }
t function periodicUpdateRTC() {
t   updateMultiple(formUpdate, updateRTC);
t   setTimeout(periodicUpdateRTC, formUpdate.period);
t }
t </script></head>
t <body onload="periodicUpdateRTC()">
t <form action="datatime.cgi" method="post" name="datatime">
t Periodic:<periodicUpdateRTC()>
i pg_header.inc
t <h2 align=center><br>System Time and Date (RTC)</h2>
t <p><font size="2">
t This page displays the current <b>Date and Time</b> obtained from the <b>RTC Module</b>.
t The information is updated in real time.<br><br>
t </font></p>
t <table border=0 width=99%><font size="3">
t <tr bgcolor=#aaccff>
t <th width=40%>Item</th>
t <th width=60%>Value</th>
t </tr>
t <tr>
t <td><img src=pabb.gif>Current Time</td>
t <td align="center">
c h 1 <input type="text" readonly style="background-color: transparent" size="20" id="current_time" value="%s">
t </td>
t </tr>
t <tr>
t <td><img src=pabb.gif>Current Date</td>
t <td align="center">
c h 2 <input type="text" readonly style="background-color: transparent" size="20" id="current_date" value="%s">
t </td>
t </tr>
t </font></table>
t <p align=center><font size="2">
t The RTC is updated automatically every second.
t </font></p>
i pg_footer.inc
.
