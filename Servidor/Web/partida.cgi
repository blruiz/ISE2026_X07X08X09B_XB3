t <html><head><title>JUGANDO</title>
i pg_header.inc
t <script language=JavaScript type="text/javascript" src="xml_http.js"></script>
t <script language=JavaScript type="text/javascript">
t var pnts_estadoUpdate = new periodicObj("partida.cgx", 100);
t function update_puntos_estado() {
t  updateMultiple(pnts_estadoUpdate, function() {
t   var estado = document.getElementById("accion_go").value;
t   var pnts = document.getElementById("puntos").value;
t   var xtime = document.getElementById("current_time").value;
t   var xdate = document.getElementById("current_date").value;
t   var cons = document.getElementById("ad_value").value;
t   document.getElementById("estado_ago").value = estado;
t   document.getElementById("puntuacion").value = pnts;
t   document.getElementById("display_time").value = xtime;
t   document.getElementById("display_date").value = xdate;
t   document.getElementById("ad_volts").value = cons + " mA";
t    if (estado === "PAUSA") {
t      document.getElementById("pause_overlay").style.display = "block";
t      document.getElementById("bloque_estado").style.display = "none";
t      document.getElementById("btn_volver").style.display = "none";
t      document.getElementById("game_over_text").style.display = "none";
t      return;
t    }
t    if (estado === "DERROTA") {
t       document.getElementById("bloque_estado").style.display = "none";
t       document.getElementById("btn_volver").style.display = "inline-block";
t       document.getElementById("game_over_text").style.display = "block";
t       document.getElementById("pause_overlay").style.display = "none";
t      return;
t     }
t      document.getElementById("pause_overlay").style.display = "none";
t      document.getElementById("bloque_estado").style.display = "block";
t      document.getElementById("btn_volver").style.display = "none";
t      document.getElementById("game_over_text").style.display = "none";
t  });
t }
t function periodicUpdate() {
t   update_puntos_estado();
t   setTimeout(periodicUpdate, pnts_estadoUpdate.period);
t }
t </script>
t </head>
t <body onload="periodicUpdate()" style="
t   margin:0;
t   padding:0;
t   background:black;
t   background-image:linear-gradient(rgba(255,255,255,0.03) 1px, transparent 1px);
t   background-size:100% 3px;
t   font-family:Arial, sans-serif;
t   color:white;
t ">
t <div id="pause_overlay" style="
t  display:none;
t  position:fixed;
t  top:0; left:0;
t  width:100%; height:100%;
t  background:rgba(0,0,0,0.8);
t  color:#00ffff;
t  font-family:'Impact';
t  text-align:center;
t  z-index:9999;
t  padding-top:15%;
t ">
t  <div style="font-size:80px;">PAUSA</div>
t  <div style="
t    font-size:22px;
t    margin-top:20px;
t    color:#cccccc;
t    font-family:Arial;
t  ">
t    pulse el botón para seguir jugando
t  </div>
t </div>
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
t <input type="hidden" id="accion_go" value="">
t <input type="hidden" id="puntos" value="">
# <input type="hidden" id="accion_hidden" value="%s">
t <!-- TÍTULO ARCADE -->
t <h2 align=center style="
t   font-family:'Impact';
t   font-size:48px;
t   color:#00ffff;
t   margin-top:10px;
t ">
t JUGANDO
t </h2>
t <!-- ESTADO + PUNTOS -->
t <table border=0 width="100%" style="margin-top:20px; color:white;">
t <tr>
t   <td align="center" style="padding:10px;">
t   <div id="bloque_estado">
t     <span style="font-family:'Impact'; font-size:28px; color:#ffcc00;">ESTADO</span><br>
t     <input type="text" readonly style=" background-color:transparent; color:#00ffff; border:none; 
t     c n 1 font-size:60px; text-align:center;" size="20" id="estado_ago" value="%s">
t    </div>
t   </td>
t </tr>
t <tr>
t   <td align="center" style="padding:10px;">
t     <span style="font-family:'Impact'; font-size:28px; color:#ffcc00;">PUNTOS</span><br>
t     <input type="text" readonly style=" background-color:transparent; color:#00ff00; border:none; 
t     c n 2 font-size:60px; text-align:center;" size="20" id="puntuacion" value="%u">
t   </td>
t </tr>
t <tr>
t   <td align="center">
t     <img id="btn_volver" src="bton_volver_a_jugar.jpg"
t     onclick="location='nuevapartida.cgi'"
t     style="
t        display:none;
t        width:150px;
t        cursor:pointer;
t        margin-top:146px;
t        image-rendering:pixelated;
t     ">
t <div id="game_over_text" style="
t     display:none;
t    font-family:'Impact';
t    font-size:60px;
t   color:#ff0000;
t    text-shadow:0 0 20px #ff0000;
t    margin-top:20px;
t ">
t    GAME OVER
t </div>
t   </td>
t </tr>
t </table>
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
