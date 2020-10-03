var app = new Vue({
    el: '#app',
    data: {
        status: ""
    },
    delimiters: ['[[',']]'],    //vue跟jinja的語法會衝突
    created: function(){
        this.InitROSConnection();
    },
    methods: {
        InitROSConnection: function(){
            var wsHost = $("meta[name='ws_host']").attr("content");
            var wsPort = $("meta[name='ws_port']").attr("content");
            
            var ros = new ROSLIB.Ros({
                url : "ws://"+wsHost+":"+wsPort
            });
        
            ros.on('connection', function() {
                this.status = "connected";
            }.bind(this));
        
            ros.on('error', function(error) {
                this.status = err;
            }.bind(this));
        
            ros.on('close', function() {
                this.status = "closed";
            }.bind(this));

            var listener = new ROSLIB.Topic({
                ros : ros,
                name : "/car_state",
                messageType : 'std_msgs/String'
            });
        
            listener.subscribe(function(message) {
                this.status = message.data;
            }.bind(this));

        }
    }
});