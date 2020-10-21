var app = new Vue({
    el: '#app',
    data: {
        connectState: "",
        status:  {
            robotState: "",
            gps: {lat: null, lng: null},
            usFL:-1, usF:-1, usFR:-1,
            usBL:-1, usB:-1, usBR:-1
        },
        topic: {
            carState:  {name: "/car_state", type:"std_msgs/String",instance:null},
            fsmState: {name: "/fsm/state",type:"std_msgs/String",instance:null},
            fsmEvent: {name: "/fsm/event",type:"std_msgs/String",instance:null},
            carCmd: {name:"/car_cmd",type:"geometry_msgs/Twist",instance:null},
            frontRGB:  {name: "/apriltag/detected/compressed", type:"sensor_msgs/CompressedImage",instance:null},
            sideRGB:  {name: "/camera/color/image_raw/compressed", type:"sensor_msgs/CompressedImage",instance:null},
            sideDepth:  {name: "/camera/aligned_depth_to_color/image_raw/compressedDepth", type:"sensor_msgs/CompressedImage",instance:null},
        },
        service: {
            followTagGetParam:  {name: "/followTag/getParam", type:"vapaa_cruiser/followTagGetParam",instance:null},
            followTagSetParam:  {name: "/followTag/setParam", type:"vapaa_cruiser/followTagSetParam",instance:null},
        },
        imageData:{
            frontRGB: "static/image/logo.png",
            sideRGB: "static/image/logo.png",
            sideDepth: "static/image/logo.png"
        },
        trajectory:{
            map: null,
            path: null,
            marker: null,
            lastUpdate: null
        },
        joystick: {
            touch: false,
            x: 0,
            y: 0
        },
        followTag: {
            tagID: null,
            distance: null,
            tolerance: null
        },
        loading: true
    },
    delimiters: ['[[',']]'],    //vue跟jinja的語法會衝突
    created: function(){
        this.InitROSConnection();
        this.InitMap();
        this.loading = false;
    },
    methods: {
        InitROSConnection: function(){
            var wsHost = $("meta[name='ws_host']").attr("content");
            var wsPort = $("meta[name='ws_port']").attr("content");
            
            var ros = new ROSLIB.Ros({
                url : "ws://"+wsHost+":"+wsPort
            });
        
            ros.on('connection', function() {
                this.connectState = "connected";
            }.bind(this));
        
            ros.on('error', function(error) {
                this.connectState = error;
            }.bind(this));
        
            ros.on('close', function() {
                this.connectState = "closed";
            }.bind(this));

            //subscribe topics
            this.topic.carState.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.carState.name,
                messageType : this.topic.carState.type
            });
            this.topic.carState.instance.subscribe(function(msg) {
                var arr = msg.data.split(",")
                this.status.gps.lat = parseFloat(arr[0]);
                this.status.gps.lng = parseFloat(arr[1]);
                this.status.usFL = parseFloat(arr[2]);
                this.status.usF = parseFloat(arr[3]);
                this.status.usFR = parseFloat(arr[4]);
                this.status.usBL = parseFloat(arr[5]);
                this.status.usB = parseFloat(arr[6]);
                this.status.usBR = parseFloat(arr[7]);

                //var t = spacetime.now();
                //this.status.gps.lat = 23.5+Math.sin(t.millisecond());
                //this.status.gps.lng = 121+Math.cos(t.millisecond());

                this.UpdateTrajectory();
            }.bind(this));

            this.topic.fsmState.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.fsmState.name,
                messageType : this.topic.fsmState.type
            });
            this.topic.fsmState.instance.subscribe(function(msg) {
                this.status.robotState = msg.data;
            }.bind(this));

            this.topic.frontRGB.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.frontRGB.name,
                messageType : this.topic.frontRGB.type
            });
            this.topic.frontRGB.instance.subscribe(function(msg) {
                this.imageData.frontRGB = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

            this.topic.sideRGB.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.sideRGB.name,
                messageType : this.topic.sideRGB.type
            });
            this.topic.sideRGB.instance.subscribe(function(msg) {
                this.imageData.sideRGB = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

            this.topic.sideDepth.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.sideDepth.name,
                messageType : this.topic.sideDepth.type
            });
            this.topic.sideDepth.instance.subscribe(function(msg) {
                this.imageData.sideDepth = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

            //publish topics
            this.topic.fsmEvent.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.fsmEvent.name,
                messageType : this.topic.fsmEvent.type
            });

            this.topic.carCmd.instance = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.carCmd.name,
                messageType : this.topic.carCmd.type
              });

            setInterval(function(){
                if(this.status.robotState != "JOYSTICK_CONTROL") return;
                var forward = 0, turn = 0;
                if(this.joystick.touch){
                    var joystick = $("#joystick");
                    forward = (0.5-this.joystick.y/joystick.height())*2;
                    turn = (this.joystick.x/joystick.width()-0.5)*2;
                }
                var twist = new ROSLIB.Message({
                    linear : {x : forward, y : 0, z : 0},
                    angular : {x : 0, y : 0, z : turn}
                  });
                  //console.log([forward,turn]);
                  this.topic.carCmd.instance.publish(twist);
            }.bind(this), 30);
            
            //services
            this.service.followTagGetParam.instance = new ROSLIB.Service({
                ros: ros,
                name: this.service.followTagGetParam.name,
                serviceType : this.service.followTagGetParam.type
            });
            
            var request = new ROSLIB.ServiceRequest({});
            this.service.followTagGetParam.instance.callService(request, function(result) {
                this.followTag.tagID = result.tagID;
                this.followTag.distance = result.distance;
                this.followTag.tolerance = result.tolerance;
            }.bind(this));

            this.service.followTagSetParam.instance = new ROSLIB.Service({
                ros: ros,
                name: this.service.followTagSetParam.name,
                serviceType : this.service.followTagSetParam.type
            });

        },
        InitMap: function(){
            Vue.nextTick(function(){
                this.trajectory.map = L.map("map").setView([23.682094, 120.7764642], 7);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    attribution: '<a href="https://www.openstreetmap.org/">OSM</a>',
                    maxZoom: 19,
                }).addTo(this.trajectory.map);
                L.control.scale().addTo(this.trajectory.map);
            }.bind(this));
            
        },
        Logout: function(){
            window.location.href="/logout";
        },
        FSMTransition: function(event){
            var msg = new ROSLIB.Message({
                data : event
            });
            this.topic.fsmEvent.instance.publish(msg);
        },
        StartJoystick: function(evt){
            $("#mainContent").css("overflow","hidden");
            if(evt.type == "touchstart"){
                evt = evt.touches[0];
            }
            this.joystick.x = evt.pageX-$("#joystick").offset().left;
            this.joystick.y = evt.pageY-$("#joystick").offset().top;
            this.joystick.touch = true;
        },
        StopJoystick: function(evt){
            $("#mainContent").css("overflow","");
            this.joystick.touch = false;
        },
        MoveJoystick: function(evt){
            if(!this.joystick.touch) return;
            if(evt.type == "touchmove"){
                evt = evt.touches[0];
            }
            var joystick = $("#joystick");
            var x = evt.pageX-joystick.offset().left;
            var y = evt.pageY-joystick.offset().top;
            var halfW = joystick.width()*0.5;
            var halfH = joystick.height()*0.5;
            //restrict pos in circle
            var r = Math.sqrt((x-halfW)*(x-halfW)+(y-halfH)*(y-halfH));
            if( r > halfW){
                x = (x-halfW)*halfW/r+halfW;
                y = (y-halfH)*halfH/r+halfH;
            }
            this.joystick.x = x;
            this.joystick.y = y;
        },
        GetJoystickPos: function(){
            var indicator = $(".touch-indicator");
            var x = "left: "+(this.joystick.x-indicator.width()*0.5)+"px;";
            var y = "top: "+(this.joystick.y-indicator.height()*0.5)+"px;";
            return x+y;
        },
        UpdateFollowTagParam: function(){
            var request = new ROSLIB.ServiceRequest({
                tagID: this.followTag.tagID,
                distance: this.followTag.distance,
                tolerance: this.followTag.tolerance
            });
            this.service.followTagSetParam.instance.callService(request, function(result) {
                if(result.success){
                    alert("更新成功");
                }
                else{
                    alert("更新失敗");
                }
            }.bind(this));
        },
        UpdateTrajectory: function(){
            if(this.status.gps.lat < -90 || this.status.gps.lat > 90 || this.status.gps.lng < -180 || this.status.gps.lng > 180){
                //console.log("invalid gps: lat="+this.status.gps.lat+", lng="+this.status.gps.lng);
                return;
            }
            //update marker
            if(this.trajectory.marker){
                this.trajectory.marker.setLatLng(this.status.gps);
            }
            else{
                this.trajectory.marker = L.marker(this.status.gps);
                this.trajectory.marker.addTo(this.trajectory.map);
            }
            //update trajectory
            if(!this.trajectory.path){
                this.trajectory.path = L.polyline([], {color: "red"}).addTo(this.trajectory.map);
                this.trajectory.path.addLatLng(this.status.gps);
                this.trajectory.lastUpdate = spacetime.now();
            }
            else{
                var curTime = spacetime.now();
                if(this.trajectory.lastUpdate.diff(curTime,"second") >= 10){
                    this.trajectory.path.addLatLng(this.status.gps);
                    this.trajectory.lastUpdate = curTime;
                }
                
            }
        }
    }
});