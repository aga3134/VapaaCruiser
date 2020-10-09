var app = new Vue({
    el: '#app',
    data: {
        connectState: "",
        action: "",
        status:  {
            pos: {lat: null, lng: null},
            usFL:-1, usF:-1, usFR:-1,
            usBL:-1, usB:-1, usBR:-1
        },
        topic: {
            carState:  {name: "/car_state", type:"std_msgs/String"},
            frontRGB:  {name: "/image/compressed", type:"sensor_msgs/CompressedImage"},
            sideRGB:  {name: "/camera/color/image_raw/compressed", type:"sensor_msgs/CompressedImage"},
            sideDepth:  {name: "/camera/aligned_depth_to_color/image_raw/compressedDepth", type:"sensor_msgs/CompressedImage"},
        },
        imageData:{
            frontRGB: "static/image/logo.png",
            sideRGB: "static/image/logo.png",
            sideDepth: "static/image/logo.png"
        },
        trajectory:{
            map: null,
            marker: null
        }
    },
    delimiters: ['[[',']]'],    //vue跟jinja的語法會衝突
    created: function(){
        this.InitROSConnection();
        this.InitMap();
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
            var carState = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.carState.name,
                messageType : this.topic.carState.type
            });
            carState.subscribe(function(msg) {
                var arr = msg.data.split(",")
                this.status.pos.lat = parseFloat(arr[0]);
                this.status.pos.lng = parseFloat(arr[1]);
                this.status.usFL = parseFloat(arr[2]);
                this.status.usF = parseFloat(arr[3]);
                this.status.usFR = parseFloat(arr[4]);
                this.status.usBL = parseFloat(arr[5]);
                this.status.usB = parseFloat(arr[6]);
                this.status.usBR = parseFloat(arr[7]);

                if(this.trajectory.marker){
                    this.trajectory.marker.setLatLng(this.status.pos);
                }
                else{
                    this.trajectory.marker = L.marker(this.status.pos);
                    this.trajectory.marker.addTo(this.trajectory.map);
                }
            }.bind(this));

            var frontRGB = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.frontRGB.name,
                messageType : this.topic.frontRGB.type
            });
            frontRGB.subscribe(function(msg) {
                this.imageData.frontRGB = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

            var sideRGB = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.sideRGB.name,
                messageType : this.topic.sideRGB.type
            });
            sideRGB.subscribe(function(msg) {
                this.imageData.sideRGB = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

            var sideDepth = new ROSLIB.Topic({
                ros : ros,
                name : this.topic.sideDepth.name,
                messageType : this.topic.sideDepth.type
            });
            sideDepth.subscribe(function(msg) {
                this.imageData.sideDepth = "data:image/jpeg;base64,"+msg.data;
            }.bind(this));

        },
        InitMap: function(){
            Vue.nextTick(function(){
                this.trajectory.map = L.map("map").setView([23.682094, 120.7764642], 7);
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    attribution: '<a href="https://www.openstreetmap.org/">OSM</a>',
                    maxZoom: 19,
                }).addTo(this.trajectory.map);
            }.bind(this));
            
        },
        Logout: function(){
            window.location.href="/logout";
        },
        ChangeAction: function(action){
            this.action = action;
        }
    }
});