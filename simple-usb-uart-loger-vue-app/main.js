import './style.css'
import javascriptLogo from './javascript.svg'
import viteLogo from '/logo.svg'
import { setupCounter } from './counter.js'
import { Serial } from './serial.js'
import Toastify from 'toastify-js'

console.log("serial", Serial);

// setupCounter(document.querySelector('#counter'))

const log = document.querySelector('#log');
const connectButton = document.querySelector('#btn-connect');
const btn_clear = document.querySelector('#btn-clear');

let port;

const statusDisplay = document.querySelector('#status');

function toast(text) {
  Toastify({
    text: text,
    duration: 5000,
    close: true,
    gravity: "top", // `top` or `bottom`
    position: "left", // `left`, `center` or `right`
    // stopOnFocus: true, // Prevents dismissing of toast on hover
    style: {
      // background: "linear-gradient(to right, #00b09b, #96c93d)",
    },
    onClick: function(){} // Callback after click
  }).showToast();
  statusDisplay.textContent = text;
}

if(0) {
Serial.getPorts().then(ports => {
  if (ports.length === 0) {
    toast('No device found.');
  } else {
    toast('Connecting...');
    port = ports[0];
    connect();
  }
});
}

// console.log("connectButton", connectButton);

function addLogLine(logtext) {
  // let timestamp = +Date.now() - start_timestamp;
  // const delta = timestamp - last_timestamp;
  // last_timestamp = timestamp;
  // const hours = Math.floor(timestamp / 3600000);
  // const minutes = Math.floor(timestamp / 60000) % 60;
  // const seconds = Math.floor(timestamp / 1000) % 60;
  // const milliseconds = timestamp % 1000;
  const line = document.createElement('p');
  // line.innerHTML = `[${hours}:${minutes}:${seconds}.${milliseconds}+${delta}ms] ${logtext}`;
  line.innerHTML = `${logtext}`;
  line.className = (logtext.startsWith('>'))?'line-in':'line-out';
  log.appendChild(line);

  // var objDiv = document.getElementById("your_div");
  const autoscroll = document.querySelectorAll("input#autoscroll")[0].checked;
  if(autoscroll) {
    log.scrollTop = log.scrollHeight;
  }
}

// function addLine(linesId, text) {
//   var senderLine = document.createElement("div");
//   senderLine.className = 'line';
//   var textnode = document.createTextNode(text);
//   senderLine.appendChild(textnode);
//   document.getElementById(linesId).appendChild(senderLine);
//   return senderLine;
// }

let currentReceiverLine;

function appendLines(linesId, text) {
  console.log("appendLines", linesId, text);
  const lines = text.split('\r');
    for (let i = 0; i < lines.length; i++) {
      addLogLine(lines[i]);
      // currentReceiverLine = addLine(linesId, lines[i]);
    }
  // if (currentReceiverLine) {
  //   currentReceiverLine.innerHTML =  currentReceiverLine.innerHTML + lines[0];
  //   for (let i = 1; i < lines.length; i++) {
  //     currentReceiverLine = addLine(linesId, lines[i]);
  //   }
  // } else {
  //   for (let i = 0; i < lines.length; i++) {
  //     currentReceiverLine = addLine(linesId, lines[i]);
  //   }
  // }
}


function connect() {
  port.connect().then(() => {
    statusDisplay.textContent = '';
    connectButton.textContent = 'Disconnect';
    document.querySelector('#baudrate').removeAttribute('disabled');

    setTimeout(() => {
      let baudrate = document.querySelector('#baudrate').value;
      baudrate = baudrate?baudrate:'6'; // 115200
      console.log("send baudrate", baudrate);
      port.send(new TextEncoder('utf-8').encode('\r\n'+baudrate));
    }, 200);

    port.onReceive = data => {
      let textDecoder = new TextDecoder();
      console.log("Reveived", textDecoder.decode(data));
      // if (data.getInt8() === 13) {
      //   currentReceiverLine = null;
      // } else {
        appendLines('receiver_lines', textDecoder.decode(data));
        // addLogLine(textDecoder.decode(data));
      // }
    };
    port.onReceiveError = error => {
      console.error(error);
    };
    // port.onC
  }, error => {
    console.error(error);
    toast(error);
  });
}


connectButton.addEventListener('click', function() {
  if (port) {
    port.disconnect();
    connectButton.textContent = 'Connect';
    statusDisplay.textContent = '';
    document.querySelector('#baudrate').setAttribute('disabled', 'disabled');
    port = null;
  } else {
    Serial.requestPort().then(selectedPort => {
      port = selectedPort;
      connect();
    }).catch(error => {
      console.error(error);
      toast(error);
    });
  }
});

document.querySelector('#baudrate').addEventListener('change', function(e) {
  console.log("baudrate", e.target.value);
  if(port) {
    port.send(new TextEncoder('utf-8').encode(e.target.value));
  }
});



let start_timestamp = +Date.now();
let last_timestamp = 0;


// for (let i = 0; i < 50; i++) {
//   const line_in = document.createElement('p');
//   line_in.innerHTML = `> Line ${i}`;
//   line_in.className = 'line-in';
//   log.appendChild(line_in);

//   const line_out = document.createElement('p');
//   line_out.innerHTML = `< Line ${i}`;
//   line_out.className = 'line-out';
//   log.appendChild(line_out);
// }
