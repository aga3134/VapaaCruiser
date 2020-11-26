L.SvgIcon = L.DivIcon.extend({
    options: {
        svgID: ""
    },
    initialize: function(options) {
        this.angle = 0;
        options.className = "";
        var svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.id = options.svgID;
        svg.style.width = '100%';
        svg.style.height = '100%';
        options.html = svg.outerHTML;
        L.setOptions(this, options);
    },
    setAngle: function(angle){
        this.angle = angle;
        this.update();
    },
    update: function(){
        var svg = d3.select("#"+this.options.svgID);
        svg.selectAll("*").remove();
        var w = this.options.iconSize[0];
        var h = this.options.iconSize[1];
        svg.append("rect")
            .attr("transform", "rotate("+this.angle+","+(w*0.5)+","+(h*0.5)+")")
            .attr("x", 0)
            .attr("y", h*0.25)
            .attr("width", w)
            .attr("height", h*0.5)
            .style("fill", "#ff0000");
    }
});

L.svgIcon = function(options) {
    return new L.SvgIcon(options);
}

//==========================================================
L.SvgMarker = L.Marker.extend({
    setAngle: function(angle){
        this.options.icon.setAngle(angle);
    },
    update: function(){
        L.Marker.prototype.update.call(this);
        this.options.icon.update();
    }
});

L.svgMarker = function(latlng, options) {
    return new L.SvgMarker(latlng, options);
}



