"use strict"; //2012-06-23 http://alexandre.alapetite.fr

function escapeHtml(text)
{
  return text.replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#039;");
}

var fs = require('fs');
var os = require('os');
var path = require('path');
var util = require('util');

var serverSignature = 'Node.js / Debian ' + os.type() + ' ' + os.release() + ' ' + os.arch() + ' / Raspberry Pi';

function done(request, response)
{
  util.log(request.connection.remoteAddress + '\t' + response.statusCode + '\t"' + request.method + ' ' + request.url + '"\t"' +
    request.headers['user-agent'] + '"\t"' + request.headers['accept-language'] + '"\t"' + request.headers['referer'] + '"');
}

function serve400(request, response)
{
  response.writeHead(400,
  {
    'Content-Type': 'text/html; charset=UTF-8',
    'Date': (new Date()).toUTCString(),
    'Server': serverSignature
  });
  response.end('<!DOCTYPE html>\n\
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en-GB" lang="en-GB">\n\
<head>\n\
<meta charset="UTF-8" />\n\
<title>400 Bad Request</title>\n\
</head>\n\
<body>\n\
<h1>Bad Request</h1>\n\
<p>Your browser sent a request that this server could not understand.</p>\n\
</body>\n\
</html>\n\
');
  done(request, response);
}

function serve404(request, response, requestUrl)
{//When a static file is not found
  response.writeHead(404,
  {
    'Content-Type': 'text/html; charset=UTF-8',
    'Date': (new Date()).toUTCString(),
    'Server': serverSignature
  });
  response.end('<!DOCTYPE html>\n\
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en-GB" lang="en-GB">\n\
<head>\n\
<meta charset="UTF-8" />\n\
<title>404 Not Found</title>\n\
</head>\n\
<body>\n\
<h1>Not Found</h1>\n\
<p>The requested <abbr title="Uniform Resource Locator">URL</abbr> <kbd>' +
  escapeHtml(requestUrl.pathname) + '</kbd> was not found on this server.</p>\n\
</body>\n\
</html>\n\
');
  done(request, response);
}

function serveStaticFile(request, response, requestUrl)
{
  var myPath = '.' + requestUrl.pathname;
  if (myPath && (/^\.\/[a-z0-9_-]+\.[a-z]{2,4}$/i).test(myPath) && (!(/\.\./).test(myPath)))
    fs.stat(myPath, function (err, stats)
    {
      if ((!err) && stats.isFile())
      {
        var ext = path.extname(myPath);
        var mimes = { '.css': 'text/css', '.html': 'text/html', '.ico': 'image/x-icon', '.jpg': 'image/jpeg',
          '.js': 'application/javascript', '.json': 'application/json', '.png': 'image/png', '.txt': 'text/plain', '.xml': 'application/xml', '.manifest': 'text/cache-manifest'   };
        var modifiedDate = new Date(stats.mtime).toUTCString();
        if (modifiedDate === request.headers['if-modified-since'])
        {
          response.writeHead(304,
          {
            'Content-Type': ext && mimes[ext] ? mimes[ext] : 'application/octet-stream',
            'Date': (new Date()).toUTCString()
          });
          response.end();
        }
        else
        {
          response.writeHead(200,
          {
            'Content-Type': ext && mimes[ext] ? mimes[ext] : 'application/octet-stream',
            'Content-Length': stats.size,
            'Cache-Control': 'public, max-age=86400',
            'Date': (new Date()).toUTCString(),
            'Last-Modified': modifiedDate,
            'Server': serverSignature
          });
          fs.createReadStream(myPath).pipe(response);
        }
        done(request, response);
      }
      else serve404(request, response, requestUrl);
    });
  else serve404(request, response, requestUrl);
}

function serveHome(request, response, requestUrl)
{
  var now = new Date();
  response.writeHead(200,
  {
    'Content-Type': 'text/html; charset=UTF-8',
    'Date': now.toUTCString(),
    'Server': serverSignature
  });
  response.end('<!DOCTYPE html>\n\
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en-GB" lang="en-GB">\n\
<head>\n\
<meta charset="UTF-8" />\n\
<title>Test of Node.js on Raspberry Pi</title>\n\
<meta name="robots" content="noindex" />\n\
<meta name="viewport" content="initial-scale=1.0,width=device-width" />\n\
</head>\n\
<body>\n\
<pre>\n\
Hello ' + request.connection.remoteAddress + '!\n\
This is <a href="http://nodejs.org/" rel="external">Node.js</a> on <a href="http://www.raspberrypi.org/" rel="external">Raspberry Pi</a> :-)\n\
It is now ' + now.toISOString() + '.\n\
</pre>\n\
<ul>\n\
<li><a href="index.js">Source code</a></li>\n\
<li><a href="http://alexandre.alapetite.fr/doc-alex/raspberrypi-nodejs-arduino/">Explanations</a></li>\n\
</ul>\n\
</body>\n\
</html>\n\
');
  done(request, response);
}

var http = require('http');
var url = require('url');

var server = http.createServer(function (request, response)
{
  if (request && request.url)
  {
    var requestUrl = url.parse(request.url);
    switch (requestUrl.pathname)
    {
      case '/': serveHome(request, response, requestUrl); break;
      default: serveStaticFile(request, response, requestUrl); break;
    }
  }
  else serve400(request, response);
}); 

var net = require('net');

var HOST = '127.0.0.1';
var PORT = 8001;

var client = new net.Socket();

// Add a 'close' event handler for the client socket
client.on('close', function() {
    console.log('Connection closed');
});
             
client.on('error', function (e) {
        console.log('Error connecting...');
});

var io = require('socket.io').listen(server);

// Quand on client se connecte, on le note dans la console
io.sockets.on('connection', function (socket) {
    var n;
              
    console.log('Un client est connecté !');
    socket.emit('message',' \n');
    socket.emit('message','You\'re properly connected to the server\n');
    socket.emit('message','You can now switch the robot on\n');
              
    client.connect(PORT, HOST, function() {
        console.log('CONNECTED TO: ' + HOST + ':' + PORT);
    });
    client.setNoDelay(true);
              
    // Add a 'data' event handler for the client socket
    // data is what the server sent to this socket
    client.on('data', function(data) {
              //console.log('Message reçu : ' + data);
        var chaine = data.toString();
           
        // from server to Web interface
              
        if (chaine.indexOf('PNGT') == 0) {
              socket.emit('pongT','received pong T\n');
        } else if (chaine.indexOf('PNGF') == 0) {
              socket.emit('pongF','received pong F\n');
        } else if (chaine.indexOf('LOCF') == 0) {
              socket.emit('locF','received Locate F\n');
        } else if (chaine.indexOf('LOCT') == 0) {
              socket.emit('locT','received Locate T\n');
        } else if (chaine.indexOf('SCPTT') == 0) {
              socket.emit('scptT','received script successful\n');
        } else if (chaine.indexOf('SCPTF') == 0) {
              socket.emit('scptF','received script Failed\n');
        } else if (chaine.indexOf('TOPWF') == 0) {
              socket.emit('TopWifiF','received TopWifi F\n');
        } else if (chaine.indexOf('TOPWT') == 0) {
              socket.emit('TopWifiT','received TopWifi T\n');
        } else if (chaine.indexOf('AXY') == 0) {
            socket.emit('axy',chaine.substring(3));
        } else if (chaine.indexOf('VOLT') == 0) {
              socket.emit('volt',chaine.substring(4));
        } else if (chaine.indexOf('MEM') == 0) {
              socket.emit('mem',chaine.substring(3));
        } else if (chaine.indexOf('SR1') == 0) {
              socket.emit('sr1',chaine.substring(3));
        } else if (chaine.indexOf('SR2') == 0) {
              socket.emit('sr2',chaine.substring(3));
        } else if (chaine.indexOf('LOGF') == 0) {
              socket.emit('LogFileOK',chaine.substring(4));
        } else if (chaine.indexOf('ENG') == 0) {
              socket.emit('eng',chaine.substring(3));
        } else if (chaine.indexOf('PLOTH') >= 0) {
              n = chaine.indexOf('PLOTH');
              socket.emit('ploth',chaine.substring(n+5));
        } else if (chaine.indexOf('PLOTV') >= 0) {
              n = chaine.indexOf('PLOTV');
              socket.emit('plotv',chaine.substring(n+5));
        } else if (chaine.indexOf('DRAWMURS') >= 0) {
              n = chaine.indexOf('DRAWMURS');
              socket.emit('drawmurs',chaine.substring(n+8));
        } else if (chaine.indexOf('CLEARPLAN') >= 0) {
              socket.emit('ClearPlan',chaine.substring(n+8));
        } else if (chaine.indexOf('DRAWCOLOR') >= 0) {
              n = chaine.indexOf('DRAWCOLOR');
              socket.emit('drawcolor',chaine.substring(n+9));
        } else if (chaine.indexOf('COLOR') >= 0) {
              socket.emit('color',chaine.substring(5));
        } else {
              socket.emit('message','' + data);
        }
    });

    // from Web interface to server
       
    socket.on('cmd', function (message) {
      client.write(message);
    });

            
//    socket.on('wifiref', function (message) {
//      client.write('WREF');
//    });
//    socket.on('locate', function (message) {
//      client.write('LOCT');
//    });
    socket.on('LogFile', function (message) {
      client.write('LOGF' + message);
    });
    socket.on('config', function (message) {
      client.write('CFG');
    });
    socket.on('servo1', function (message) {
        client.write('SR1' + message);
    });
    socket.on('servo2', function (message) {
        client.write('SR2' + message);
    });
    socket.on('scan', function (message) {
            console.log('Message reçu : ' + message);
            client.write('SCN' + message);
    });
    socket.on('goto', function (message) {
            console.log('Message reçu : ' + message);
            client.write('GOTO' + message);

    });

});

server.on('error', function (e) {
    if (e.code == 'EADDRINUSE') {
            console.log('Address in use, retrying...');
            setTimeout(function () {
                        server.close();
                        server.listen(8080);
                        }, 1000);
            }
});
             
server.listen(8080);
console.log('Node.js server running at %j', server.address());
