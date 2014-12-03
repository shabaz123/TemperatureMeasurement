#!/usr/bin/env node

var app = require('http').createServer(handler)
  , io = require('socket.io').listen(app)
  , fs = require('fs');
var path = require('path');

app.listen(8081);

// variables
var child=require('child_process');
var prog;
var prog2;
var runscope=0;

var progpath='/home/pi/development/therm/';
var values = "abc";
var rfile = progpath;
var child2=require('child_process').exec;
var cprog2;
var oldinfo1='1';
var oldinfo2='2';

// HTML handler
function handler (req, res)
{
	console.log('url is '+req.url.substr(1));
	reqfile=req.url.substr(1);
//	if (reqfile != "tlog-logo.png")
//	{
//		reqfile="index.html"; // only allow this file for now
//	}
	fs.readFile(progpath+reqfile,
  function (err, data)
  {
    if (err)
    {
      res.writeHead(500);
      return res.end('Error loading index.html');
    }
    res.writeHead(200);
    res.end(data);
  });
}

function ads_adc(v)
{
	prog=child.exec(progpath+'therm'), function (error, data, stderr) {
	  values=data.toString();
	  console.log('old prog executed, values length is '+values.length);
	};
	prog.on('exit', function(code)
	{
		socket.emit('results', {measurement: values});
		console.log('old app complete, values length is '+values.length);
	});
	
}




// Socket.IO comms handling
// A bit over-the-top but we use some handshaking here
// We advertise message 'status stat:idle' to the browser once,
// and then wait for message 'action command:xyz'
// We handle the action xyz and then emit the message 'status stat:done'
io.sockets.on('connection', function (socket)
{
	socket.emit('status', {stat: 'ready'});
	
  socket.on('action', function (data)
  {
  	var dummy="dummy cmd";
  	var temp=dummy.split(" ");
  	cmd=data.command;
    console.log("command is "+cmd);
    isgettemp=0;
    isreadfile=0;
    islogstart=0;
    islogstop=0;
    ischeckstate=0;
  	if (cmd=="gettemp")
  	{
  		isgettemp=1;
  	}

  	
		if (isgettemp)
		{
			//ads_adc();
		}
		else
		{
			cmd=cmd.toString();
			temp = cmd.split(" ");
			if (temp[0]=="logstart")
			{
				islogstart=1;
			}
			else if (temp[0]=="logstop")
			{
				islogstop=1;
			}
			else if (temp[0]=="readfile")
			{
				isreadfile=1;
			}
			else if (temp[0]=="checkstate")
			{
				ischeckstate=1;
			}
		}

		if (isgettemp)
		{
			prog=child.exec('sudo '+progpath+'therm withtime', function (error, data, stderr) {
	  		values=data.toString();
	  		//console.log('retrieve complete, length is '+values.length);
	  		socket.emit('results', values);
			});
		}
		if (isreadfile)
		{
			prog=child.exec('tail -n 1 '+progpath+temp[1], function (error, data, stderr) {
	  		console.log('line is '+data);
	  		if (data.length<5) // too short
	  		{
	  			socket.emit('lastline', 'error,0,error');
	  		}
	  		else
	  		{
	  			oldinfo1=oldinfo2;
	  			oldinfo2=data;
	  			if (oldinfo1==oldinfo2)
	  			{
	  				if (oldinfo2==data)
	  				{
	  					socket.emit('lastline', 'finished');
	  				}
	  			}
	  			else
	  			{
	  				socket.emit('lastline', data);
	  			}
	  		}
			});						
		
		}
		if (islogstart)
		{
			cprog2=child2('sudo '+progpath+'therm 1 '+temp[1]+' msg "Logging..." &');
			//cprog2.stdout.on('data', function(data) {
			//});
			//cprog2.stderr.on('data', function(data) {
			//});
			//cprog2.on('close', function(code) {
			//});
			
		}
		if (islogstop)
		{
			// not implemented yet
		}
		
		if (ischeckstate)
		{
			var state_is_logging=0;
			prog=child.exec('ps -ef | grep therm', function (error, data, stderr) {
				var lines = data.toString().split('\n');
				lines.forEach(function(line) {
					if (line.indexOf(progpath+'therm')>0)
					{
						if (line.indexOf('msg')>0)
						{
							// ok, confident it is logging
							state_is_logging=1;
						}
					}
							
				});
				if (state_is_logging==1)
				{
					socket.emit('stateresult', 'logging');
				}
				else
				{
					socket.emit('stateresult', 'idle');
				}
	  		//values=data.toString();
	  		//socket.emit('results', values);
			});
		}
		
		// ret=handleCommand(cmd);
    // socket.emit('status', {stat: ret});


  }); // end of socket.on('action', function (data)

}); // end of io.sockets.on('connection', function (socket)


