(function(){const i=document.createElement("link").relList;if(i&&i.supports&&i.supports("modulepreload"))return;for(const n of document.querySelectorAll('link[rel="modulepreload"]'))f(n);new MutationObserver(n=>{for(const r of n)if(r.type==="childList")for(const t of r.addedNodes)t.tagName==="LINK"&&t.rel==="modulepreload"&&f(t)}).observe(document,{childList:!0,subtree:!0});function e(n){const r={};return n.integrity&&(r.integrity=n.integrity),n.referrerPolicy&&(r.referrerPolicy=n.referrerPolicy),n.crossOrigin==="use-credentials"?r.credentials="include":n.crossOrigin==="anonymous"?r.credentials="omit":r.credentials="same-origin",r}function f(n){if(n.ep)return;n.ep=!0;const r=e(n);fetch(n.href,r)}})();const p={getPorts:function(){return navigator.usb.getDevices().then(o=>o.map(i=>new p.Port(i)))},requestPort:function(){const o=[{vendorId:51966},{vendorId:9114},{vendorId:11914},{vendorId:12346},{vendorId:9025}];return navigator.usb.requestDevice({filters:o}).then(i=>new p.Port(i))},Port:function(o){this.device_=o,this.interfaceNumber=0,this.endpointIn=0,this.endpointOut=0}};p.Port.prototype.connect=function(){let o=()=>{this.device_.transferIn(this.endpointIn,64).then(i=>{this.onReceive(i.data),o()},i=>{this.onReceiveError(i)})};return this.device_.open().then(()=>{if(this.device_.configuration===null)return this.device_.selectConfiguration(1)}).then(()=>{var i=this.device_.configuration.interfaces;i.forEach(e=>{e.alternates.forEach(f=>{f.interfaceClass==255&&(this.interfaceNumber=e.interfaceNumber,f.endpoints.forEach(n=>{n.direction=="out"&&(this.endpointOut=n.endpointNumber),n.direction=="in"&&(this.endpointIn=n.endpointNumber)}))})})}).then(()=>this.device_.claimInterface(this.interfaceNumber)).then(()=>this.device_.selectAlternateInterface(this.interfaceNumber,0)).then(()=>this.device_.controlTransferOut({requestType:"class",recipient:"interface",request:34,value:1,index:this.interfaceNumber})).then(()=>{o()})};p.Port.prototype.disconnect=function(){return this.device_.controlTransferOut({requestType:"class",recipient:"interface",request:34,value:0,index:this.interfaceNumber}).then(()=>this.device_.close())};p.Port.prototype.send=function(o){return this.device_.transferOut(this.endpointOut,o)};var T=typeof globalThis<"u"?globalThis:typeof window<"u"?window:typeof global<"u"?global:typeof self<"u"?self:{};function x(o){return o&&o.__esModule&&Object.prototype.hasOwnProperty.call(o,"default")?o.default:o}var N={exports:{}};/*!
 * Toastify js 1.12.0
 * https://github.com/apvarun/toastify-js
 * @license MIT licensed
 *
 * Copyright (C) 2018 Varun A P
 */(function(o){(function(i,e){o.exports?o.exports=e():i.Toastify=e()})(T,function(i){var e=function(t){return new e.lib.init(t)},f="1.12.0";e.defaults={oldestFirst:!0,text:"Toastify is awesome!",node:void 0,duration:3e3,selector:void 0,callback:function(){},destination:void 0,newWindow:!1,close:!1,gravity:"toastify-top",positionLeft:!1,position:"",backgroundColor:"",avatar:"",className:"",stopOnFocus:!0,onClick:function(){},offset:{x:0,y:0},escapeMarkup:!0,ariaLive:"polite",style:{background:""}},e.lib=e.prototype={toastify:f,constructor:e,init:function(t){return t||(t={}),this.options={},this.toastElement=null,this.options.text=t.text||e.defaults.text,this.options.node=t.node||e.defaults.node,this.options.duration=t.duration===0?0:t.duration||e.defaults.duration,this.options.selector=t.selector||e.defaults.selector,this.options.callback=t.callback||e.defaults.callback,this.options.destination=t.destination||e.defaults.destination,this.options.newWindow=t.newWindow||e.defaults.newWindow,this.options.close=t.close||e.defaults.close,this.options.gravity=t.gravity==="bottom"?"toastify-bottom":e.defaults.gravity,this.options.positionLeft=t.positionLeft||e.defaults.positionLeft,this.options.position=t.position||e.defaults.position,this.options.backgroundColor=t.backgroundColor||e.defaults.backgroundColor,this.options.avatar=t.avatar||e.defaults.avatar,this.options.className=t.className||e.defaults.className,this.options.stopOnFocus=t.stopOnFocus===void 0?e.defaults.stopOnFocus:t.stopOnFocus,this.options.onClick=t.onClick||e.defaults.onClick,this.options.offset=t.offset||e.defaults.offset,this.options.escapeMarkup=t.escapeMarkup!==void 0?t.escapeMarkup:e.defaults.escapeMarkup,this.options.ariaLive=t.ariaLive||e.defaults.ariaLive,this.options.style=t.style||e.defaults.style,t.backgroundColor&&(this.options.style.background=t.backgroundColor),this},buildToast:function(){if(!this.options)throw"Toastify is not initialized";var t=document.createElement("div");t.className="toastify on "+this.options.className,this.options.position?t.className+=" toastify-"+this.options.position:this.options.positionLeft===!0?(t.className+=" toastify-left",console.warn("Property `positionLeft` will be depreciated in further versions. Please use `position` instead.")):t.className+=" toastify-right",t.className+=" "+this.options.gravity,this.options.backgroundColor&&console.warn('DEPRECATION NOTICE: "backgroundColor" is being deprecated. Please use the "style.background" property.');for(var s in this.options.style)t.style[s]=this.options.style[s];if(this.options.ariaLive&&t.setAttribute("aria-live",this.options.ariaLive),this.options.node&&this.options.node.nodeType===Node.ELEMENT_NODE)t.appendChild(this.options.node);else if(this.options.escapeMarkup?t.innerText=this.options.text:t.innerHTML=this.options.text,this.options.avatar!==""){var d=document.createElement("img");d.src=this.options.avatar,d.className="toastify-avatar",this.options.position=="left"||this.options.positionLeft===!0?t.appendChild(d):t.insertAdjacentElement("afterbegin",d)}if(this.options.close===!0){var c=document.createElement("button");c.type="button",c.setAttribute("aria-label","Close"),c.className="toast-close",c.innerHTML="&#10006;",c.addEventListener("click",(function(v){v.stopPropagation(),this.removeElement(this.toastElement),window.clearTimeout(this.toastElement.timeOutValue)}).bind(this));var a=window.innerWidth>0?window.innerWidth:screen.width;(this.options.position=="left"||this.options.positionLeft===!0)&&a>360?t.insertAdjacentElement("afterbegin",c):t.appendChild(c)}if(this.options.stopOnFocus&&this.options.duration>0){var l=this;t.addEventListener("mouseover",function(v){window.clearTimeout(t.timeOutValue)}),t.addEventListener("mouseleave",function(){t.timeOutValue=window.setTimeout(function(){l.removeElement(t)},l.options.duration)})}if(typeof this.options.destination<"u"&&t.addEventListener("click",(function(v){v.stopPropagation(),this.options.newWindow===!0?window.open(this.options.destination,"_blank"):window.location=this.options.destination}).bind(this)),typeof this.options.onClick=="function"&&typeof this.options.destination>"u"&&t.addEventListener("click",(function(v){v.stopPropagation(),this.options.onClick()}).bind(this)),typeof this.options.offset=="object"){var h=n("x",this.options),m=n("y",this.options),y=this.options.position=="left"?h:"-"+h,L=this.options.gravity=="toastify-top"?m:"-"+m;t.style.transform="translate("+y+","+L+")"}return t},showToast:function(){this.toastElement=this.buildToast();var t;if(typeof this.options.selector=="string"?t=document.getElementById(this.options.selector):this.options.selector instanceof HTMLElement||typeof ShadowRoot<"u"&&this.options.selector instanceof ShadowRoot?t=this.options.selector:t=document.body,!t)throw"Root element is not defined";var s=e.defaults.oldestFirst?t.firstChild:t.lastChild;return t.insertBefore(this.toastElement,s),e.reposition(),this.options.duration>0&&(this.toastElement.timeOutValue=window.setTimeout((function(){this.removeElement(this.toastElement)}).bind(this),this.options.duration)),this},hideToast:function(){this.toastElement.timeOutValue&&clearTimeout(this.toastElement.timeOutValue),this.removeElement(this.toastElement)},removeElement:function(t){t.className=t.className.replace(" on",""),window.setTimeout((function(){this.options.node&&this.options.node.parentNode&&this.options.node.parentNode.removeChild(this.options.node),t.parentNode&&t.parentNode.removeChild(t),this.options.callback.call(t),e.reposition()}).bind(this),400)}},e.reposition=function(){for(var t={top:15,bottom:15},s={top:15,bottom:15},d={top:15,bottom:15},c=document.getElementsByClassName("toastify"),a,l=0;l<c.length;l++){r(c[l],"toastify-top")===!0?a="toastify-top":a="toastify-bottom";var h=c[l].offsetHeight;a=a.substr(9,a.length-1);var m=15,y=window.innerWidth>0?window.innerWidth:screen.width;y<=360?(c[l].style[a]=d[a]+"px",d[a]+=h+m):r(c[l],"toastify-left")===!0?(c[l].style[a]=t[a]+"px",t[a]+=h+m):(c[l].style[a]=s[a]+"px",s[a]+=h+m)}return this};function n(t,s){return s.offset[t]?isNaN(s.offset[t])?s.offset[t]:s.offset[t]+"px":"0px"}function r(t,s){return!t||typeof s!="string"?!1:!!(t.className&&t.className.trim().split(/\s+/gi).indexOf(s)>-1)}return e.lib.init.prototype=e.lib,e})})(N);var E=N.exports;const O=x(E);console.log("serial",p);const k=document.querySelector("#log"),b=document.querySelector("#btn-connect");document.querySelector("#btn-clear");let u;const g=document.querySelector("#status");function C(o){O({text:o,duration:5e3,close:!0,gravity:"top",position:"left",style:{},onClick:function(){}}).showToast(),g.textContent=o}function P(){u.connect().then(()=>{g.textContent="",b.textContent="Disconnect",u.onReceive=o=>{let i=new TextDecoder;console.log(i.decode(o)),o.getInt8()===13?currentReceiverLine=null:I(i.decode(o))},u.onReceiveError=o=>{console.error(o)}},o=>{console.error(o),C(o)})}b.addEventListener("click",function(){u?(u.disconnect(),b.textContent="Connect",g.textContent="",u=null):p.requestPort().then(o=>{u=o,P()}).catch(o=>{console.error(o),C(o)})});let _=+Date.now(),w=0;function I(o){let i=+Date.now()-_;const e=document.createElement("p"),f=i-w;w=i;const n=Math.floor(i/36e5),r=Math.floor(i/6e4)%60,t=Math.floor(i/1e3)%60,s=i%1e3;e.innerHTML=`[${n}:${r}:${t}.${s}+${f}ms] ${o}`,k.appendChild(e)}