/*
    The main changes here (other than UI) are to make the video be one-way from
    the server to the browser.   And no audio.

    Note - this is left as a separate js file in order to keep somewhat
    consistent w/ the aiortc server example.    There are multiple changes,
    but generally the same flow.

    The main UI control is at the bottom of index.html
*/

// get DOM elements
var dataChannelLog = document.getElementById('data-channel'),
    iceConnectionLog = document.getElementById('ice-connection-state'),
    iceGatheringLog = document.getElementById('ice-gathering-state'),
    signalingLog = document.getElementById('signaling-state'),

    cbUseStun = document.getElementById('use-stun'),

    objVideo = document.getElementById('video'),
    // objAudio = document.getElementById('audio'),
    txtOfferSdp = document.getElementById('offer-sdp'),
    txtAnswerSdp = document.getElementById('answer-sdp');

// peer connection
var pc = null;

// data channel
var dc = null, dcOpen = false, dcWatchdog = null;

// support adding a timestamp to watchdog messages
var time_start = null;
function current_stamp() {
    if (time_start === null) {
        time_start = new Date().getTime();
        return 0;
    } else {
        return new Date().getTime() - time_start;
    }
}

function negotiate() {
    pc.addTransceiver('video', {direction: 'recvonly'});
    // pc.addTransceiver('audio', {direction: 'recvonly'});
    return pc.createOffer().then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        // wait for ICE gathering to complete
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        var offer = pc.localDescription;
        txtOfferSdp.textContent = offer.sdp;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then(function(response) {
        return response.json();
    }).then(function(answer) {
        txtAnswerSdp.textContent = answer.sdp;
        return pc.setRemoteDescription(answer);
    }).catch(function(e) {
        console.log("negotiate error: " + e);
        alert(e);
    });
}


/***** 
 * Added in data channel access
 *****/

 function dataChannelSend(jmessage) {
    if (dcOpen) {
        msg = JSON.stringify(jmessage);
        dc.send(msg);
        // console.log("sent message: " + msg);
    }
 }

 function dataChannelReceive(msg) {
    try {
        jmsg = JSON.parse(msg);
    } catch (e) {
        console.log("failed to json parse message: " + msg);
        alert(e);       // should really never happen?
        return;
    }

    // handle message type
    // console.log("received message: " + jmsg);
    if ('robot-status' in jmsg) {
        // placeholder for now
    } else if ('watchdog-server' in jmsg) {
        // currently do nothing
        // console.log("watchdog-server: " + jmsg['watchdog-server']);
    } else {
        console.log("unknown message type: " + jmsg['type']);
    }
 }

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (cbUseStun.checked) {
        config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    }

    pc = new RTCPeerConnection(config);

    // Add in some debugging logging
    pc.addEventListener('icegatheringstatechange', function() {
        iceGatheringLog.textContent += ' -> ' + pc.iceGatheringState;
    }, false);
    iceGatheringLog.textContent = pc.iceGatheringState;

    pc.addEventListener('iceconnectionstatechange', function() {
        iceConnectionLog.textContent += ' -> ' + pc.iceConnectionState;
    }, false);
    iceConnectionLog.textContent = pc.iceConnectionState;

    pc.addEventListener('signalingstatechange', function() {
        signalingLog.textContent += ' -> ' + pc.signalingState;
    }, false);
    signalingLog.textContent = pc.signalingState;

    // connect audio / video
    pc.addEventListener('track', function(evt) {
        if (evt.track.kind == 'video')
            objVideo.srcObject = evt.streams[0];
        else
            objAudio.srcObject = evt.streams[0];
    });


    // options:                            
    // Ordered, reliable - default
    // Unordered, no retransmissions
    // Unordered, 500ms lifetime
    var parameters = {"ordered": true};
    dc = pc.createDataChannel('chat', parameters);
    dc.onclose = function() {
        dataChannelLog.textContent += ' -> close';
        dcOpen = false;
        clearInterval(dcWatchdog);
    };
    dataChannelLog.textContent += ' -> new';

    dc.onopen = function() {
        dataChannelLog.textContent += ' -> open';
        dcOpen = true;
        // And a watchdog timer
        dcWatchdog = setInterval(function() {
            var message = {'watchdog-browser': current_stamp()};
            dataChannelSend(message);
        }, 10000);
    };
    dc.onmessage = function(evt) {
        dataChannelReceive(evt.data);
    };

    negotiate();
}

function stop() {
    if (dc) {       
        dc.close();     // close data channel
    }

    // close transceivers
    if (pc.getTransceivers) {
        pc.getTransceivers().forEach(function(transceiver) {
            if (transceiver.stop) {
                transceiver.stop();
            }
        });
    }

    // close local audio / video
    pc.getSenders().forEach(function(sender) {
        // with our one way setup, seems like sender.track can be null - makes sense I guess
        if (sender.track) {
            sender.track.stop();
        }
    });

    // close peer connection
    setTimeout(function() {
        pc.close();
    }, 500);
}
