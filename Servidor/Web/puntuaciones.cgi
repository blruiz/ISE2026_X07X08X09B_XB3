t <html><head><title>RANKING</title>
i pg_header.inc
t </head>
t <body style="
t   margin:0;
t   padding:0;
t   background:black;
t   background-image:linear-gradient(rgba(255,255,255,0.03) 1px, transparent 1px);
t   background-size:100% 3px;
t   font-family:Arial, sans-serif;
t   color:white;">
t <table align=center width=90% style="
t   max-width:900px;
t   margin-top:20px;
t   border:4px solid #00ffff;
t   border-radius:10px;
t   box-shadow:0 0 20px #00ffff;
t   background:#000000cc;
t   padding:20px;">
t <tr><td>
t <h2 align=center style="
t   font-family:'Impact';
t   font-size:48px;
t   color:#00ffff;
t   margin-top:10px;
t   letter-spacing: 2px;">
t TOP 10
t </h2>
t <table border=0 width=100% style="
t   border-collapse: collapse;
t   color: white;
t   font-size: 18px;">
t   <thead>
t     <tr style="
t       border-bottom: 3px solid #ff00ff; 
t       font-family: 'Impact'; 
t       font-size: 22px; 
t       color: #ff00ff;">
t       <th width=30% style="padding: 10px;">JUGADOR</th>
t       <th width=20% style="padding: 10px;">PUNTOS</th>
t       <th width=50% style="padding: 10px;">FECHA Y HORA</th>
t     </tr>
t   </thead>
t   <tbody style="font-family: monospace; font-size: 18px; color: #ffffff;">
c c
t   </tbody>
t </table>
t <p align=center style="margin-top:35px;">
t </p>
t <form action="puntuaciones.cgi" method="post">
t  <p align=center style="margin-top:20px;">
t    <input type="submit" name="rst" value="Reset" style="
t      font-family:'Impact';
t      font-size:24px;
t      padding:10px 20px;
t      border:3px solid #00ff00;
t      background:black;
t      color:#00ff00;
t      border-radius:8px;">
t  </p>
t </form>
t </td></tr>
t </table>
t </body>
t </html>
i pg_footer.inc
.