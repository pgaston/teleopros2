// get DOM elements
var dataChannelLog = document.getElementById('data-channel'),
    iceConnectionLog = document.getElementById('ice-connection-state'),
    iceGatheringLog = document.getElementById('ice-gathering-state'),
    signalingLog = document.getElementById('signaling-state'),

    cbUseStun = document.getElementById('use-stun'),

    objVideo = document.getElementById('video'),
    objAudio = document.getElementById('audio'),
    txtOfferSdp = document.getElementById('offer-sdp'),
    txtAnswerSdp = document.getElementById('answer-sdp');


// peer connection
var pc = null;

// data channel
var dc = null, dcOpen = false, dcWatchdog = null;

function createPeerConnection() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (cbUseStun.checked) {
        config.iceServers = [{urls: ['stun:stun.l.google.com:19302']}];
    }

    pc = new RTCPeerConnection(config);

    // console.log("pc: " + pc);

    // register some listeners to help debugging
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


    return pc;
}

function negotiate() {
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
        var codec;

        // codec = document.getElementById('audio-codec').value;
        // if (codec !== 'default') {
        //     offer.sdp = sdpFilterCodec('audio', codec, offer.sdp);
        // }

        // codec = document.getElementById('video-codec').value;
        // if (codec !== 'default') {
        //     offer.sdp = sdpFilterCodec('video', codec, offer.sdp);
        // }

        console.log("Send offer.type: " + offer.type);

        txtOfferSdp.textContent = offer.sdp;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
                // video_transform: document.getElementById('video-transform').value
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


/***** Added in data channel access
 *****/
 function dataChannelOpen() {
    return dcOpen;
 }

 function dataChannelSend(jmessage) {
    if (dcOpen) {
        msg = JSON.stringify(jmessage);
        dc.send(msg);
        // console.log("sent message: " + msg);
    }
 }

 function dataChannelReceive(msg) {
    jmsg = JSON.parse(msg);
    // console.log("received message: " + jmsg);

    if ('watchdog-server' in jmsg) {
        // 
        // console.log("watchdog-server: " + jmsg['watchdog-server']);
    } else {
        console.log("unknown message type: " + jmsg['type']);
    }
 }

function start() {
    pc = createPeerConnection();

    var time_start = null;
    function current_stamp() {
        if (time_start === null) {
            time_start = new Date().getTime();
            return 0;
        } else {
            return new Date().getTime() - time_start;
        }
    }

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
        // dataChannelLog.textContent += '< ' + evt.data + '\n';
        dataChannelReceive(evt.data);
    };


    // if (document.getElementById('use-datachannel').checked) {
    //     var parameters = JSON.parse(document.getElementById('datachannel-parameters').value);

    //     dc = pc.createDataChannel('chat', parameters);
    //     dc.onclose = function() {
    //         dataChannelLog.textContent += ' -> close';
    //         dcOpen = false;
    //         clearInterval(dcWatchdog);
    //     };
    //     dataChannelLog.textContent += ' -> new';

    //     dc.onopen = function() {
    //         dataChannelLog.textContent += ' -> open';
    //         dcOpen = true;
    //         // And a watchdog timer
    //         dcWatchdog = setInterval(function() {
    //             var message = {'watchdog-browser': current_stamp()};
    //             dataChannelSend(message);
    //         }, 10000);
    //     };
    //     dc.onmessage = function(evt) {
    //         // dataChannelLog.textContent += '< ' + evt.data + '\n';
    //         dataChannelReceive(evt.data);
    //     };
    // }

    var constraints = {
        audio: false,   // no audio, for now
        video: true,     // video yes, but only as a reader, not creator
        // offerToReceiveAudio: false, 
        // offerToReceiveVideo: true
    };

    // if (document.getElementById('use-video').checked) {
    //     var resolution = document.getElementById('video-resolution').value;
    //     if (resolution) {
    //         resolution = resolution.split('x');
    //         constraints.video = {
    //             width: parseInt(resolution[0], 0),
    //             height: parseInt(resolution[1], 0)
    //         };
    //     } else {
    //         constraints.video = true;
    //     }
    // }


    // from https://stackoverflow.com/questions/50002099/webrtc-one-way-video-call
    pc.addTransceiver('video');
    // this step seems to be optional:
    pc.getTransceivers().forEach(t => t.direction = 'recvonly');
    negotiate();
    // or
    // remoteStream = new MediaStream();


    // negotiate();

    // if (constraints.audio || constraints.video) {
    //     // if (constraints.video) {
    //     //     document.getElementById('media').style.display = 'block';
    //     // }
    //     navigator.mediaDevices.getUserMedia(constraints).then(function(stream) {
    //         stream.getTracks().forEach(function(track) {
    //             pc.addTrack(track, stream);
    //         });
    //         return negotiate();
    //     }, function(err) {
    //         alert('Could not acquire media: ' + err);
    //     });
    // } else {
    //     negotiate();
    // }

}

function stop() {
    // close data channel
    if (dc) {
        dc.close();
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
        // with our one way setup, seems like sender.track can be null - makes sense
        if (sender.track) {
            sender.track.stop();
        }
    });

    // close peer connection
    setTimeout(function() {
        pc.close();
        // btnStart.style.display = 'block';
    }, 500);
}

function sdpFilterCodec(kind, codec, realSdp) {
    var allowed = []
    var rtxRegex = new RegExp('a=fmtp:(\\d+) apt=(\\d+)\r$');
    var codecRegex = new RegExp('a=rtpmap:([0-9]+) ' + escapeRegExp(codec))
    var videoRegex = new RegExp('(m=' + kind + ' .*?)( ([0-9]+))*\\s*$')
    
    var lines = realSdp.split('\n');

    var isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var match = lines[i].match(codecRegex);
            if (match) {
                allowed.push(parseInt(match[1]));
            }

            match = lines[i].match(rtxRegex);
            if (match && allowed.includes(parseInt(match[2]))) {
                allowed.push(parseInt(match[1]));
            }
        }
    }

    var skipRegex = 'a=(fmtp|rtcp-fb|rtpmap):([0-9]+)';
    var sdp = '';

    isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var skipMatch = lines[i].match(skipRegex);
            if (skipMatch && !allowed.includes(parseInt(skipMatch[2]))) {
                continue;
            } else if (lines[i].match(videoRegex)) {
                sdp += lines[i].replace(videoRegex, '$1 ' + allowed.join(' ')) + '\n';
            } else {
                sdp += lines[i] + '\n';
            }
        } else {
            sdp += lines[i] + '\n';
        }
    }

    return sdp;
}

function escapeRegExp(string) {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'); // $& means the whole matched string
}
