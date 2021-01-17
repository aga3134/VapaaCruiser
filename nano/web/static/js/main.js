var app = new Vue({
    el: '#app',
    data: {
        connectState: "",
        ros: null,
        status:  {
            robotState: "",
            angle: 0,
            gps: {lat: null, lng: null},
            preGPS: {lat:null, lng: null},
            usFL:{dist: -1, indicator:""},
            usF:{dist: -1, indicator:""},
            usFR:{dist: -1, indicator:""},
            usBL:{dist: -1, indicator:""},
            usB:{dist: -1, indicator:""},
            usBR:{dist: -1, indicator:""},
            posReady: false
        },
        topic: {
            carState:{name:"/car_state", type:"std_msgs/String",instance:null},
            navState:{name:"/nav_state", type:"vapaa_cruiser/NavState",instance:null},
            fsmState:{name:"/fsm/state",type:"std_msgs/String",instance:null},
            fsmEvent:{name:"/fsm/event",type:"std_msgs/String",instance:null},
            carCmd: {name:"/car_cmd",type:"geometry_msgs/Twist",instance:null},
            frontRGB:{name:"/apriltag_repub"},
            sideRGB:{name:"/camera/color/image_raw"},
            sideRGB_yolov4:{name:"/yolov4_repub"},
            sideRGB_yolov5:{name:"/yolov5_repub"},
            sideDepth:{name:"/depth_process_repub"},
        },
        service: {
            followTagGetParam:  {name: "/followTag/getParam", type:"vapaa_cruiser/FollowTagGetParam",instance:null},
            followTagSetParam:  {name: "/followTag/setParam", type:"vapaa_cruiser/FollowTagSetParam",instance:null},
            storeFront: {name: "/imageStore/front", type:"vapaa_cruiser/TriggerWithInfo",instance:null},
            storeSide: {name: "/imageStore/side", type:"vapaa_cruiser/TriggerWithInfo",instance:null},
            autoNavStart: {name: "/autoNavigation/start", type:"std_srvs/TriggerWithInfo",instance:null},
            autoNavStop: {name: "/autoNavigation/stop", type:"std_srvs/Trigger",instance:null},
            autoNavPause: {name: "/autoNavigation/pause", type:"std_srvs/Trigger",instance:null},
            autoNavSetLoop: {name: "/autoNavigation/setLoop", type:"std_srvs/SetBool",instance:null},
        },
        imageData:{
            frontRGB: "static/image/logo.png",
            sideRGB: "static/image/logo.png",
        },
        sideRGBSelect: "side",    //side,yolov4,yolov5,depth
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
        commutag:{
            apiKey: null,
            dataset: null
        },
        navigation: {
            map: null,
            posMarker: null,
            targetMarker: null,
            pathLine: null,
            openPathSelect: false,
            pathList: [],
            pathHash: {},
            selectPathID: null,
            targetIndex: 0,
            curPath: null,
            loop: false,
            pause: false
        },
        autoDrive: {
            curForward: 0, curTurn: 0,
            targetForward: 0, targetTurn: 0,
            errForward: 0, errTurn: 0,
            errSumForward: 0, errSumTurn: 0,
            forwardP: 1, forwardI: 0, forwardD: 0,
            turnP: 1, turnI: 0, turnD: 0,
            adjustByUS: true,
        },
        pathEditor: {
            openAddPt: false,
            openSavePath: false,
            addPt:{
                lat: -9999,
                lng: -9999,
                saveImage: false,
                uploadImage: false
            },
            previewMap: null,
            previewPath: null,
            highlightIndex: -1,
            highlightMarker: null,
            path: {id:null, name:"", ptArr:[]}
        },
        openSetting: false,
        loading: true
    },
    delimiters: ['[[',']]'],    //vue跟jinja的語法會衝突
    created: function(){
        this.InitROSConnection();
        Vue.nextTick(function(){
            this.navigation.map = this.InitMap("map");
        }.bind(this));
        this.loading = false;

        $.get("/setting",function(result){
            if(result.status != "ok") return;
            this.commutag.dataset = result.data.dataset;
            this.commutag.apiKey = result.data.apiKey;
        } .bind(this));

        toastr.options = {
            "positionClass": "toast-top-center",
            "timeOut": "2500",
        };
    },
    methods: {
        InitROSConnection: function(){
            var wsHost = $("meta[name='ws_host']").attr("content");
            var wsPort = $("meta[name='ws_port']").attr("content");
            
            this.ros = new ROSLIB.Ros({
                url : "ws://"+wsHost+":"+wsPort
            });
        
            this.ros.on('connection', function() {
                this.connectState = "connected";
            }.bind(this));
        
            this.ros.on('error', function(error) {
                this.connectState = error;
            }.bind(this));
        
            this.ros.on('close', function() {
                this.connectState = "closed";
            }.bind(this));

            //subscribe topics
            this.topic.carState.instance = new ROSLIB.Topic({
                ros : this.ros,
                name : this.topic.carState.name,
                messageType : this.topic.carState.type
            });
            this.topic.carState.instance.subscribe(function(msg) {
                var arr = msg.data.split(",");
                //位置使用navState topic的數值，carState的lat,lng是直接從gps sensor輸出，跳動大且沒有旋轉角度
                this.status.usFL.dist = parseFloat(arr[2]);
                this.status.usF.dist = parseFloat(arr[3]);
                this.status.usFR.dist = parseFloat(arr[4]);
                this.status.usBL.dist = parseFloat(arr[5]);
                this.status.usB.dist = parseFloat(arr[6]);
                this.status.usBR.dist = parseFloat(arr[7]);

                function ComputeIndicator(dist){
                    var alertDist = 500, warnDist = 700;    //單位: mm
                    if(dist < alertDist) return "danger";
                    else if(dist < warnDist) return "warn";
                    else return "safe";
                }
                this.status.usFL.indicator = ComputeIndicator(this.status.usFL.dist);
                this.status.usF.indicator = ComputeIndicator(this.status.usF.dist);
                this.status.usFR.indicator = ComputeIndicator(this.status.usFR.dist);
                this.status.usBL.indicator = ComputeIndicator(this.status.usBL.dist);
                this.status.usB.indicator = ComputeIndicator(this.status.usB.dist);
                this.status.usBR.indicator = ComputeIndicator(this.status.usBR.dist);

                //this.AutoDrive();
                this.UpdateNavigation();
            }.bind(this));

            this.topic.navState.instance = new ROSLIB.Topic({
                ros : this.ros,
                name : this.topic.navState.name,
                messageType : this.topic.navState.type
            });
            this.topic.navState.instance.subscribe(function(msg){
                this.navigation.selectPathID = msg.pathID;
                this.navigation.loop = msg.loop;
                this.navigation.pause = msg.pause;
                this.navigation.targetIndex = msg.targetIndex;

                if(this.CheckGPSValid(msg.lat,msg.lng)){
                    this.status.gps.lat = msg.lat;
                    this.status.gps.lng = msg.lng;
                    this.status.angle = msg.angle;
                    //依gps位置設定地圖中心，若之前已設過就不再設定
                    if(!this.status.posReady){
                        this.GoToCurrentPos();
                        this.status.posReady = true;
                    }
                }
	    }.bind(this));

            this.topic.fsmState.instance = new ROSLIB.Topic({
                ros : this.ros,
                name : this.topic.fsmState.name,
                messageType : this.topic.fsmState.type
            });
            this.topic.fsmState.instance.subscribe(function(msg) {
                this.status.robotState = msg.data;
                switch(this.status.robotState){
                    case "JOYSTICK_CONTROL":
                        //手機版搖桿控制時切換到全螢幕，避免drag時網址列一下出現一下不見影響操控
                        if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)){
                            this.EnterFullscreen();
                        }
                        break;
                    case "FOLLOW_TAG":
                        break;
                    case "AUTO_NAVIGATION":
                        break;
                }
                console.log("robot state: "+this.status.robotState);
            }.bind(this));

            this.UpdateImageSubscribe();

            //publish topics
            this.topic.fsmEvent.instance = new ROSLIB.Topic({
                ros : this.ros,
                name : this.topic.fsmEvent.name,
                messageType : this.topic.fsmEvent.type
            });

            this.topic.carCmd.instance = new ROSLIB.Topic({
                ros : this.ros,
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
                ros: this.ros,
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
                ros: this.ros,
                name: this.service.followTagSetParam.name,
                serviceType : this.service.followTagSetParam.type
            });

            this.service.storeFront.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.storeFront.name,
                serviceType : this.service.storeFront.type
            });

            this.service.storeSide.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.storeSide.name,
                serviceType : this.service.storeSide.type
            });

            this.service.autoNavStart.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.autoNavStart.name,
                serviceType : this.service.autoNavStart.type
            });

            this.service.autoNavStop.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.autoNavStop.name,
                serviceType : this.service.autoNavStop.type
            });

            this.service.autoNavPause.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.autoNavPause.name,
                serviceType : this.service.autoNavPause.type
            });

            this.service.autoNavSetLoop.instance = new ROSLIB.Service({
                ros: this.ros,
                name: this.service.autoNavSetLoop.name,
                serviceType : this.service.autoNavSetLoop.type
            });

        },
        UpdateImageSubscribe: function(){
            var wsHost = $("meta[name='ws_host']").attr("content");
	    var url = "http://"+wsHost+":8080/stream?topic="
            
            //用web video server的影像，直接訂閱image topic會一直lag
            this.imageData["frontRGB"] = url+this.topic.frontRGB.name;
            switch(this.sideRGBSelect){
                case "side":
                    this.imageData["sideRGB"] = url+this.topic.sideRGB.name;
                    break;
                case "yolov4":
                    this.imageData["sideRGB"] = url+this.topic.sideRGB_yolov4.name;
                    break;
                case "yolov5":
                    this.imageData["sideRGB"] = url+this.topic.sideRGB_yolov5.name;
                    break;
                case "depth":
                    this.imageData["sideRGB"] = url+this.topic.sideDepth.name;
                    break;
            }

        },
        InitMap: function(id,pos,zoom){
            if(!pos) pos = [23.9652,120.9674];
            if(!zoom) zoom = 19;
            
            var map = L.map(id,{
                zoomControl: false
            }).setView(pos, zoom);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '<a href="https://www.openstreetmap.org/">OSM</a>',
                maxZoom: 19,
            }).addTo(map);
            L.control.scale().addTo(map);

            return map;
        },
        Logout: function(){
            if(confirm("確定登出？")){
                window.location.href="/logout";
            }
        },
        FSMTransition: function(event){
            console.log("transition event: "+event);
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
            
            //restrict pos in circle
            /*var halfW = joystick.width()*0.5;
            var halfH = joystick.height()*0.5;
            var r = Math.sqrt((x-halfW)*(x-halfW)+(y-halfH)*(y-halfH));
            if( r > halfW){
                x = (x-halfW)*halfW/r+halfW;
                y = (y-halfH)*halfH/r+halfH;
            }*/
            //restrict pos in rectangle
            if(x < 0) x = 0;
            else if(x >= joystick.width()) x = joystick.width();
            if(y < 0) y = 0;
            else if(y >= joystick.height()) y = joystick.height();

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
                tagID: parseInt(this.followTag.tagID),
                distance: parseInt(this.followTag.distance),
                tolerance: parseInt(this.followTag.tolerance)
            });
            this.service.followTagSetParam.instance.callService(request, function(result) {
                if(result.success){
                    toastr.success("更新成功");
                }
                else{
                    toastr.error("更新失敗");
                }
            }.bind(this));
        },
        SaveSetting: function(){
            var csrfToken = $('meta[name=csrf_token]').attr('content')
            var data = {
                dataset: this.commutag.dataset,
                apiKey: this.commutag.apiKey
            };
            $.ajax({
                type: "POST",
                url: "/setting",
                data: data,
                headers: {"X-CSRF-Token": csrfToken},
                success: function(result){
                    if(result.status == "ok"){
                        toastr.success("更新成功");
                    }
                    else{
                        toastr.error("更新失敗");
                    }
                    this.openSetting = false;
                }.bind(this)
            });
        },
        TriggerImageStore: function(dir,saveImage,uploadImage,showMessage){
            var service = null;
            switch(dir){
                case "front":
                    service = this.service.storeFront.instance;
                    break;
                case "side":
                    service = this.service.storeSide.instance;
                    break;
            }
            var info = {
                dataset: this.commutag.dataset,
                apiKey: this.commutag.apiKey,
                saveImage: saveImage,
                uploadImage: uploadImage
            };
            if(this.CheckGPSValid(this.status.gps.lat,this.status.gps.lng)){
                info.lat = this.status.gps.lat;
                info.lng = this.status.gps.lng;
            }
            var request = new ROSLIB.ServiceRequest({
                info: JSON.stringify(info)
            });
            service.callService(request, function(result) {
                //console.log(result);
                if(showMessage){
                    if(result.success){
                        if(saveImage) toastr.success("儲存成功");
                        if(uploadImage) toastr.success("上傳成功");
                    }
                    else{
                        if(saveImage) toastr.error("儲存失敗");
                        if(uploadImage) toastr.error("上傳失敗");
                    }
                }
            }.bind(this));
        },
        ChangeSideCameraTopic: function(){
            this.imageData.sideRGB = "static/image/logo.png";
            this.UpdateImageSubscribe();
        },
        CheckGPSValid: function(lat,lng){
            if(lat == null || lng == null) return false;
            if(isNaN(lat) || isNaN(lng)) return false;
            if(lat < -90 || lat > 90 || lng < -180 || lng > 180){
                return false;
            }
            else return true;
        },
        OpenAddPt: function(){
            this.pathEditor.addPt.lat = this.status.gps.lat;
            this.pathEditor.addPt.lng = this.status.gps.lng;
            this.pathEditor.openAddPt = true;
        },
        AddPosToPath: function(){
            if(!this.CheckGPSValid(this.pathEditor.addPt.lat,this.pathEditor.addPt.lng)){
                return toastr.error("gps座標錯誤");
            }
            var pt = {};
            pt.lat = this.pathEditor.addPt.lat;
            pt.lng = this.pathEditor.addPt.lng;
            pt.saveImage = this.pathEditor.addPt.saveImage;
            pt.uploadImage = this.pathEditor.addPt.uploadImage;
            this.pathEditor.path.ptArr.push(pt);

            this.pathEditor.addPt.saveImage = false;
            this.pathEditor.addPt.uploadImage = false;
            this.pathEditor.openAddPt = false;
            toastr.success("已將點位加入路徑");
        },
        MovePathPtUp: function(i){
            if(i < 1 || i >= this.pathEditor.path.ptArr.length) return;
            var temp = this.pathEditor.path.ptArr[i]
            this.pathEditor.path.ptArr[i] = this.pathEditor.path.ptArr[i-1];
            this.pathEditor.path.ptArr[i-1] = temp;
            this.UpdatePreviewPath();
        },
        MovePathPtDown: function(i){
            if(i < 0 || i >= this.pathEditor.path.ptArr.length-1) return;
            var temp = this.pathEditor.path.ptArr[i]
            this.pathEditor.path.ptArr[i] = this.pathEditor.path.ptArr[i+1];
            this.pathEditor.path.ptArr[i+1] = temp;
            this.UpdatePreviewPath();
        },
        DeletePathPt: function(i){
            if(i < 0 || i >= this.pathEditor.path.ptArr.length) return;
            if(confirm("確定刪除此點位？")){
                this.pathEditor.path.ptArr.splice(i,1);
            }
            this.UpdatePreviewPath();
        },
        EditPath: function(index){
            this.pathEditor.openSavePath = true;
            if(!this.pathEditor.previewMap ){
                Vue.nextTick(function(){
                    this.pathEditor.previewMap = this.InitMap("pathPreview");
                    this.UpdatePreviewPath();
                }.bind(this));
            }
            else this.UpdatePreviewPath();
        },
        SavePath: function(){
            if(this.pathEditor.path.name == ""){
                return toastr.error("請輸入路徑名稱");
            }
            if(this.pathEditor.path.ptArr.length < 2){
                return toastr.error("請至少加入兩個點位");
            }

            var csrfToken = $('meta[name=csrf_token]').attr('content')
            var data = {
                path: JSON.stringify(this.pathEditor.path),
            };
            if(!this.pathEditor.path.id){
                $.ajax({
                    type: "POST",
                    url: "/path/create",
                    data: data,
                    headers: {"X-CSRF-Token": csrfToken},
                    success: function(result){
                        if(result.status == "ok"){
                            toastr.success("新增成功");
                            this.ClearPath();
                        }
                        else{
                            toastr.error("新增失敗");
                        }
                    }.bind(this)
                });
            }
            else{
                $.ajax({
                    type: "POST",
                    url: "/path/edit",
                    data: data,
                    headers: {"X-CSRF-Token": csrfToken},
                    success: function(result){
                        if(result.status == "ok"){
                            toastr.success("更新成功");
                            this.ClearPath();
                        }
                        else{
                            toastr.error("更新失敗");
                        }
                    }.bind(this)
                });
            }
        },
        ResetPath: function(){
            if(confirm("重設後無法復原，確定重設路徑？")){
                this.ClearPath();
            }
        },
        ClearPath: function(){
            this.pathEditor.path.name = "";
            this.pathEditor.path.ptArr = [];
            this.pathEditor.openSavePath = false;
            this.UpdatePreviewPath();
        },
        HighlightPos: function(i){
            this.pathEditor.highlightIndex = i;
            this.UpdatePreviewPath();
        },
        UpdatePreviewPath: function(){
            if(this.pathEditor.previewPath){
                this.pathEditor.previewMap.removeLayer(this.pathEditor.previewPath);
            }
            if(this.pathEditor.highlightMarker){
                this.pathEditor.previewMap.removeLayer(this.pathEditor.highlightMarker);
            }
            this.pathEditor.previewPath = L.polyline([], {color: "blue"}).addTo(this.pathEditor.previewMap);
            var minLat = 9999, maxLat = -9999;
            var minLng = 9999, maxLng = -9999;
            for(var i=0;i<this.pathEditor.path.ptArr.length;i++){
                var pt = this.pathEditor.path.ptArr[i];
                this.pathEditor.previewPath.addLatLng({lat:pt.lat,lng:pt.lng});
                if(pt.lat < minLat) minLat = pt.lat;
                if(pt.lat > maxLat) maxLat = pt.lat;
                if(pt.lng < minLng) minLng = pt.lng;
                if(pt.lng > maxLng) maxLng = pt.lng;

                if(this.pathEditor.highlightIndex == i){
                    this.pathEditor.highlightMarker = L.marker([pt.lat,pt.lng]).addTo(this.pathEditor.previewMap); 
                }
            }
            if(minLat != 9999 && maxLat != -9999 && minLng != 9999 && maxLng != -9999){
                this.pathEditor.previewMap.fitBounds([
                    [minLat, minLng],
                    [maxLat, maxLng]
                ]);
            }
            
        },
        UpdateNavigation: function(){
            if(this.CheckGPSValid(this.status.gps.lat,this.status.gps.lng)){
                //update pos marker
                if(this.navigation.posMarker){
                    this.navigation.posMarker.setAngle(this.status.angle);
                    this.navigation.posMarker.setLatLng(this.status.gps);
                }
                else{
                    //this.navigation.posMarker = L.marker(this.status.gps).addTo(this.navigation.map);
                    var icon = L.svgIcon({
                        svgID: "carPos",
                        iconSize: [25,25],
                    });
                    this.navigation.posMarker = L.svgMarker(this.status.gps, {icon:icon}).addTo(this.navigation.map);
                }
            }
            
            //update target marker
            if(this.navigation.targetMarker){
                this.navigation.map.removeLayer(this.navigation.targetMarker);
                this.navigation.targetMarker = null;
            }
            if(!this.navigation.curPath) return;
            if(this.navigation.pause) return;
            if(this.navigation.curTargetIndex < 0) return;
            if(this.navigation.curTargetIndex >= this.navigation.curPath.path.ptArr.length) return;
            var target = this.navigation.curPath.path.ptArr[this.navigation.targetIndex];
            this.navigation.targetMarker = L.marker(target);
            this.navigation.targetMarker.addTo(this.navigation.map);

        },
        OpenPathSelect: function(){
            this.navigation.openPathSelect = true;
	    this.SetAutoNavPause(true);
            $.get("/path/list", function(result){
                if(result.status != "ok") return toastr.error("讀取路徑失敗");
                this.navigation.pathList = result.data;
                this.navigation.pathHash = {};
                for(var i=0;i<this.navigation.pathList.length;i++){
                    var p = this.navigation.pathList[i];
                    this.navigation.pathHash[p.id] = p;
                }
            }.bind(this));
        },
        SelectPath: function(i){
            this.navigation.curPath = null;
            this.navigation.selectPathID = null;
            this.navigation.openPathSelect = false;
            if(this.navigation.pathLine){
                this.navigation.map.removeLayer(this.navigation.pathLine);
                this.navigation.pathLine = null;
            }
            this.navigation.targetIndex = -1;
            if(i<0 || i>= this.navigation.pathList.length){
                var request = new ROSLIB.ServiceRequest({});
                this.service.autoNavStop.instance.callService(request, function(result){
                    toastr.success("取消路徑");
		}.bind(this));
	        return;
	    }

	    var pathID = this.navigation.pathList[i].id;
            var request = new ROSLIB.ServiceRequest({
                info: JSON.stringify({
		    id: pathID
		})
            });
            this.service.autoNavStart.instance.callService(request, function(result) {
                if(result.success){
		    var path = JSON.parse(result.message);
                    this.navigation.curPath = this.navigation.pathHash[path.id];

                    //add path line to map
                    var latlngs = [];
                    var arr = this.navigation.curPath.path.ptArr;
                    for(var i=0;i<arr.length;i++){
                        var pt = arr[i]
                        pt.lat = parseFloat(pt.lat);
                        pt.lng = parseFloat(pt.lng);
                        latlngs.push([pt.lat,pt.lng]);
                    }
                    this.navigation.pathLine = L.polyline(latlngs, {color: "red"});
                    this.navigation.pathLine.addTo(this.navigation.map);
                    this.navigation.map.fitBounds(this.navigation.pathLine.getBounds());
                }
                else{
                    toastr.error("選擇路徑失敗");
                }
            }.bind(this));
            
            
        },
        ResumePath: function(){
            this.navigation.openPathSelect = false;
            this.SetAutoNavPause(false);
        },
        ToggleAutoNavLoop: function(){
	    var loop = this.navigation.loop;
            var request = new ROSLIB.ServiceRequest({data: loop});
            this.service.autoNavSetLoop.instance.callService(request, function(result) {
               this.navigation.loop = loop; 
	    }.bind(this));
        },
        SetAutoNavPause: function(pause){
            var request = new ROSLIB.ServiceRequest({data: pause});
            this.service.autoNavPause.instance.callService(request, function(result) {
                this.navigation.pause = pause;
            }.bind(this));
        },
        DeletePath: function(i){
            this.navigation.curPath = null;
            this.navigation.selectIndex = -1;
            if(i<0 || i>= this.navigation.pathList.length) return;
            
            if(confirm("刪除後無法復原，確定刪除路徑？")){
                var csrfToken = $('meta[name=csrf_token]').attr('content')
                var data = {
                    pathID: this.navigation.pathList[i].id,
                };
                $.ajax({
                    type: "POST",
                    url: "/path/delete",
                    data: data,
                    headers: {"X-CSRF-Token": csrfToken},
                    success: function(result){
                        if(result.status == "ok"){
                            toastr.success("刪除成功");
                            this.navigation.pathList.splice(i,1);
                        }
                        else{
                            toastr.error("刪除失敗");
                        }
                    }.bind(this)
                });
            }
            
        },
        GetNavigationPathName: function(){
            if(!this.navigation.curPath) return "無";
            if(this.navigation.pause) return "暫停";
            var name = this.navigation.curPath.path.name;
            if(this.navigation.curTargetIndex >= this.navigation.curPath.path.ptArr.length && !this.navigation.loop){
                name += "(結束)";
            }
            return name;
        },
        AutoDrive: function(){
            if(!this.navigation.curPath) return;
            if(this.navigation.pause) return;
            if(this.navigation.curTargetIndex < 0) return;
            if(this.navigation.curTargetIndex >= this.navigation.curPath.path.ptArr.length){
                if(this.navigation.loop) this.navigation.curTargetIndex = 0;
                else return;
            }
	    
            //this.status.angle = 0;
	    //this.status.gps.lat = 23;
	    //this.status.gps.lng = 121;
            if(!this.CheckGPSValid(this.status.gps.lat,this.status.gps.lng)) return;
            //compute move command
            var target = this.navigation.curPath.path.ptArr[this.navigation.curTargetIndex];
            var curDir = [
                Math.cos(this.status.angle),
                Math.sin(this.status.angle)
            ];

            var targetDir = [
                target.lng - this.status.gps.lng,
                -(target.lat - this.status.gps.lat)
            ];
            var targetNorm = Math.sqrt(targetDir[0]*targetDir[0]+targetDir[1]*targetDir[1]);

            var tolerance = 1e-5;
	    var forwardLimit = 0.9;
	    var turnLimit = 1;
            if(targetNorm < tolerance){  //target arrival
                this.navigation.curTargetIndex++;
                return;
            }
            else{
                var invNorm = 1.0/targetNorm;
                targetDir[0] *= invNorm;
                targetDir[1] *= invNorm;
                var cross = curDir[0]*targetDir[1]-curDir[1]*targetDir[0];
                this.autoDrive.targetTurn = cross;
                this.autoDrive.targetForward = targetNorm;

                //依超音波測距調整command
                if(this.autoDrive.adjustByUS){
                    var turnForce = 0, turnScale = 1;
                    var breakForce = 0, breakScale = 1;
                    if(this.autoDrive.targetForward > 0){
                        if(this.status.usFL.dist > 0 && this.status.usFL.dist < 1000){
                            turnForce += turnScale/this.status.usFL.dist;
                        }
                        if(this.status.usFR.dist > 0 && this.status.usFR.dist < 1000){
                            turnForce -= turnScale/this.status.usFR.dist;
                        }
                        if(this.status.usF.dist > 0 && this.status.usF.dist < 1000){
                            breakForce -= breakScale/this.status.usF.dist;
                        }
                    }
                    else{
                        if(this.status.usBL.dist > 0 && this.status.usBL.dist < 1000){
                            turnForce -= turnScale/this.status.usBL.dist;
                        }
                        if(this.status.usBR.dist > 0 && this.status.usBR.dist < 1000){
                            turnForce += turnScale/this.status.usBR.dist;
                        }
                        if(this.status.usB.dist > 0 && this.status.usB.dist < 1000){
                            breakForce += breakScale/this.status.usB.dist;
                        }
                    }
                    this.autoDrive.targetTurn += turnForce;
                    this.autoDrive.targetForward += breakForce;
                }
                //limit target magnitude
                if(this.autoDrive.targetForward > forwardLimit) this.autoDrive.targetForward = forwardLimit;
                if(this.autoDrive.targetForward < -forwardLimit) this.autoDrive.targetForward = -forwardLimit;
                if(this.autoDrive.targetTurn > turnLimit) this.autoDrive.targetTurn = turnLimit;
                if(this.autoDrive.targetTurn < -turnLimit) this.autoDrive.targetTurn = -turnLimit;
            }
            //update command by PID controller
            var errForward = this.autoDrive.targetForward - this.autoDrive.curForward;
            var errTurn = this.autoDrive.targetTurn - this.autoDrive.curTurn;
            var errDiffForward = errForward - this.autoDrive.errForward;
            var errDiffTurn = errTurn - this.autoDrive.errTurn;
            this.autoDrive.errSumForward += errForward;
            this.autoDrive.errSumTurn += errTurn;

            this.autoDrive.curForward += 
                errForward*this.autoDrive.forwardP+
                this.autoDrive.errSumForward*this.autoDrive.forwardI+
                errDiffForward*this.autoDrive.forwardD;

            this.autoDrive.curTurn += 
                errTurn*this.autoDrive.turnP+
                this.autoDrive.errSumTurn*this.autoDrive.turnI+
                errDiffTurn*this.autoDrive.turnD;

            this.autoDrive.errForward = errForward;
            this.autoDrive.errTurn = errTurn;

            //limit command
            if(this.autoDrive.curForward > forwardLimit) this.autoDrive.curForward = forwardLimit;
            if(this.autoDrive.curForward < -forwardLimit) this.autoDrive.curForward = -forwardLimit;
            if(this.autoDrive.curTurn > turnLimit) this.autoDrive.curTurn = turnLimit;
            if(this.autoDrive.curTurn < -turnLimit) this.autoDrive.curTurn = -turnLimit;
            
            //console.log(this.autoDrive);
            //send command message
            var twist = new ROSLIB.Message({
                linear : {x : this.autoDrive.curForward, y : 0, z : 0},
                angular : {x : 0, y : 0, z : this.autoDrive.curTurn}
            });
            this.topic.carCmd.instance.publish(twist);

            //simulate straight move toward target
            /*if(!this.CheckGPSValid(this.status.gps.lat,this.status.gps.lng)){
                this.status.gps.lat = target.lat;
                this.status.gps.lng = target.lng;
                this.status.preGPS.lat = target.lat-0.0001;
                this.status.preGPS.lng = target.lng;
            }
            else{
                var speed = 1e-6;
                this.status.preGPS.lat = this.status.gps.lat;
                this.status.preGPS.lng = this.status.gps.lng;
                this.status.gps.lat += -targetDir[1]*speed;
                this.status.gps.lng += targetDir[0]*speed;
            }
            //update angle
            var latDiff = this.status.gps.lat-this.status.preGPS.lat;
            var lngDiff = this.status.gps.lng-this.status.preGPS.lng;
            this.status.angle = Math.atan2(-latDiff,lngDiff)*180/Math.PI;
            */

            //simulate circle move
            /*var t = spacetime.now();
            var r = 0.0001, w = 0.002*Math.PI;
            this.status.preGPS.lat = this.status.gps.lat;
            this.status.preGPS.lng = this.status.gps.lng;
            this.status.gps.lat = 23.9652+r*Math.sin(w*t.millisecond());
            this.status.gps.lng = 120.9674+r*Math.cos(w*t.millisecond());
            //update angle
            var latDiff = this.status.gps.lat-this.status.preGPS.lat;
            var lngDiff = this.status.gps.lng-this.status.preGPS.lng;
            this.status.angle = Math.atan2(-latDiff,lngDiff)*180/Math.PI;
            */

            
        },
        StopNavigation: function(){
            this.SelectPath(-1);
            this.FSMTransition("abort");
        },
        GoToCurrentPos: function(){
            if(!this.navigation.map) return;
            if(this.CheckGPSValid(this.status.gps.lat,this.status.gps.lng)){
                this.navigation.map.panTo(new L.LatLng(this.status.gps.lat,this.status.gps.lng));
            }
        },
        EnterFullscreen: function(){
            var doc = window.document;
            var docEl = doc.documentElement;
            var requestFullScreen = docEl.requestFullscreen || docEl.mozRequestFullScreen || docEl.webkitRequestFullScreen || docEl.msRequestFullscreen;
            var cancelFullScreen = doc.exitFullscreen || doc.mozCancelFullScreen || doc.webkitExitFullscreen || doc.msExitFullscreen;
            requestFullScreen.call(docEl);
        },
        ExitFullscreen: function(){
            var doc = window.document;
            var docEl = doc.documentElement;
            var requestFullScreen = docEl.requestFullscreen || docEl.mozRequestFullScreen || docEl.webkitRequestFullScreen || docEl.msRequestFullscreen;
            var cancelFullScreen = doc.exitFullscreen || doc.mozCancelFullScreen || doc.webkitExitFullscreen || doc.msExitFullscreen;
            cancelFullScreen.call(doc);
        },
        ExitManualNavigation: function(){
            if(/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent)){
                this.ExitFullscreen();
            }
            this.FSMTransition('abort');
        }
    }
});
