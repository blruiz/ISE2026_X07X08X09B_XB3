t <html><head><title>Jugador</title>
i pg_header.inc
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
t var rtc_consumoUpdate = new periodicObj("rtc_consumo.cgx", 500);
t function updateRTC_Consumo() {
t  updateMultiple(rtc_consumoUpdate, function() {
t   var xtime = document.getElementById("current_time").value;
t   var xdate = document.getElementById("current_date").value;
t   document.getElementById("display_time").value = xtime;
t   document.getElementById("display_date").value = xdate;
t   var cons = document.getElementById("ad_value").value;
t   document.getElementById("ad_volts").value = cons + " mA";
t  });
t }
t function periodicUpdate() {
t   updateRTC_Consumo();
t   setTimeout(periodicUpdate, rtc_consumoUpdate.period);
t }
t </script>
t </head>
t <!-- FONDO ARCADE COMPATIBLE -->
t <body onload="periodicUpdate()" style="
t   margin:0;
t   padding:0;
t   background:black;
t   background-image:linear-gradient(rgba(255,255,255,0.03) 1px, transparent 1px);
t   background-size:100% 3px;
t   font-family:Arial, sans-serif;
t   color:white;
t ">
t <table align=center width=90% style="
t   max-width:900px;
t   margin-top:20px;
t   border:4px solid #00ffff;
t   border-radius:10px;
t   box-shadow:0 0 20px #00ffff;
t   background:#000000cc;
t   padding:20px;
t ">
t <tr><td>
t <form action=partida.cgi method=post name=nuevapartida>
t <input type="hidden" id="current_time" value="">
t <input type="hidden" id="current_date" value="">
t <input type="hidden" id="ad_value" value="">
t <!-- TÍTULO ARCADE SIN GLOW -->
t <h2 align=center style="
t   font-family:'Impact';
t   font-size:48px;
t   color:#00ffff;
t   margin-top:10px;
t ">
t NUEVA PARTIDA
t </h2>
t <p align=center style="font-size:20px; color:#cccccc;">
t Introduce el nombre del jugador y pulsa <b>Jugar</b>
t </p>
t <input type=hidden value="player" name=pg>
t <!-- TABLA DE INPUT -->
t <table border=0 width="100%" style="margin-top:20px; color:white;">
t <tr align="center" style="background:transparent;">
t   <td style="padding:10px;">
t     <input type=text name=player_name size=20 maxlength=20 value=""
t       style="padding:5px; font-size:18px; border-radius:5px;
t       border:2px solid #00ffff; background:black; color:#00ffff;">
t   </td>
t </tr>
t </table>
#-- BOTONES ARCADE
t <p align=center style="margin-top:20px;">
t   <input type=submit name=play value="Jugar" style="
t     font-family:'Impact';
t     font-size:24px;
t     padding:10px 20px;
t     border:3px solid #00ff00;
t     background:black;
t     color:#00ff00;
t     border-radius:8px;
t   ">
t   
t   <input type=reset value="Borrar" style="
t     font-family:'Impact';
t     font-size:24px;
t     padding:10px 20px;
t     border:3px solid #ff00ff;
t     background:black;
t     color:#ff00ff;
t     border-radius:8px;
t     margin-left:20px;
t   ">
t </p>
t <!-- RTC + CONSUMO -->
t <table border="0" width="100%" style="margin-top:30px; color:white;">
t <tr>
t   <!-- Bloque vertical: hora arriba, fecha abajo -->
t   <td align="left">
t     <div style="display:flex; flex-direction:column;">
t       <!-- HORA -->
t       <input type="text" readonly
t         style="background-color:transparent; color:#00ffff; border:none;
t                font-size:30px; line-height:1.1;"
t         c e 1 size="12" id="display_time" value="%s">
t       <!-- FECHA -->
t       <input type="text" readonly
t         style="background-color:transparent; color:#00ffff; border:none;
t                font-size:24px; line-height:1.1; margin-top:5px;"
t         c e 2 size="12" id="display_date" value="%s">
t     </div>
t   </td>
t   <!-- CONSUMO (ADC) a la derecha, grande -->
t   <td align="right">
t     <input type="text" readonly
t       style="background-color:transparent; color:#00ffff; border:none;
t              font-size:30px;"
t       c e 3 size="10" id="ad_volts" value="%d mA">
t   </td>
t </tr>
t </table>
t </form>
t </td></tr>
t </table>
t </body>
t </html>
.