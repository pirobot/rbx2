/*
 Longclick Event
 Copyright (c) 2010 Petr Vostrel (http://petr.vostrel.cz/)
 Dual licensed under the MIT (MIT-LICENSE.txt)
 and GPL (GPL-LICENSE.txt) licenses.

 Version: 0.3.2
 Updated: 2010-06-22
*/
(function(a){function n(b){a.each("touchstart touchmove touchend touchcancel".split(/ /),function(d,e){b.addEventListener(e,function(){a(b).trigger(e)},false)});return a(b)}function j(b){function d(){a(e).data(h,true);b.type=f;jQuery.event.handle.apply(e,o)}if(!a(this).data(g)){var e=this,o=arguments;a(this).data(h,false).data(g,setTimeout(d,a(this).data(i)||a.longclick.duration))}}function k(){a(this).data(g,clearTimeout(a(this).data(g))||null)}function l(b){if(a(this).data(h))return b.stopImmediatePropagation()||
false}var p=a.fn.click;a.fn.click=function(b,d){if(!d)return p.apply(this,arguments);return a(this).data(i,b||null).bind(f,d)};a.fn.longclick=function(){var b=[].splice.call(arguments,0),d=b.pop();b=b.pop();var e=a(this).data(i,b||null);return d?e.click(b,d):e.trigger(f)};a.longclick={duration:500};a.event.special.longclick={setup:function(){/iphone|ipad|ipod/i.test(navigator.userAgent)?n(this).bind(q,j).bind([r,s,t].join(" "),k).bind(m,l).css({WebkitUserSelect:"none"}):a(this).bind(u,j).bind([v,
w,x,y].join(" "),k).bind(m,l)},teardown:function(){a(this).unbind(c)}};var f="longclick",c="."+f,u="mousedown"+c,m="click"+c,v="mousemove"+c,w="mouseup"+c,x="mouseout"+c,y="contextmenu"+c,q="touchstart"+c,r="touchend"+c,s="touchmove"+c,t="touchcancel"+c,i="duration"+c,g="timer"+c,h="fired"+c})(jQuery);
