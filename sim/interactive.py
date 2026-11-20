"""Run with: python sim/interactive.py"""
import math
import sys
import time
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

from call_inspector import call_text
from antenna_model import BEAM_HALF_DEG, DT, DriveTest, FlightTest, MIN_HEIGHT_GAP_M, MODES, Simulation, beam_direction, wrap_diff

INK, MUTED, LINE = "#183538", "#4d6869", "#c8d8d3"
AIR, GROUND = "#b65521", "#147b74"
FONT = "Microsoft YaHei UI" if sys.platform == "win32" else "sans-serif"
PHASES = {"NEAR":"几何对准", "SWEEP":"圆周搜索", "REFINE":"车端精扫",
          "VERIFY_PEAK":"宽网格复核", "CAL_AIR":"机端校正", "TRACK":"跟踪采样",
          "RECOVER":"局部重捕", "POINT":"指向 / 等待到位", "WAIT_DATA":"等待数据",
          "FAULT":"设备故障", "IDLE":"未使能"}
NAMES = ("机载设备", "车载设备")
FIELDS = (("lat","纬度 °"), ("lon","经度 °"), ("height","高度 m"), ("heading","航向 °"),
          ("az","实测方位 °"), ("el","实测俯仰 °"))


def beam_footprint(x, y, radius, az, el):
    """Convex outline of the finite 3D main lobe projected onto the map."""
    half = math.radians(BEAM_HALF_DEG)
    points = [(x, y)]
    for i in range(32):
        angle = i * math.tau / 32
        forward = math.cos(half)*math.cos(el) - math.sin(half)*math.sin(el)*math.cos(angle)
        right = math.sin(half)*math.sin(angle)
        points.append((x+radius*(forward*math.sin(az)+right*math.cos(az)),
                       y-radius*(forward*math.cos(az)-right*math.sin(az))))
    def boundary(sequence):
        edge = []
        for p in sequence:
            while len(edge) >= 2:
                a, b = edge[-2:]
                if (b[0]-a[0])*(p[1]-a[1]) > (b[1]-a[1])*(p[0]-a[0]):
                    break
                edge.pop()
            edge.append(p)
        return edge[:-1]
    points.sort()
    return boundary(points) + boundary(reversed(points))


class StateDiagram(tk.Frame):
    rows = (("NEAR", "SWEEP", "REFINE", "VERIFY_PEAK"),
            ("IDLE", "RECOVER", "TRACK", "CAL_AIR"),
            ("WAIT_DATA", "FAULT", "POINT"))

    def __init__(self, parent, paths):
        super().__init__(parent, bg="white")
        self.paths = paths
        self.source = tk.StringVar(value="phase")
        self.last_view = None
        bar = tk.Frame(self, bg="white")
        bar.pack(fill="x")
        tk.Label(bar, text="状态与跳转", bg="white", fg=INK,
                 font=(FONT, 10, "bold")).pack(side="left", padx=6)
        for key, title in (("phase", "控制阶段"), ("air", "机端输出"), ("ground", "车端输出")):
            ttk.Radiobutton(bar, text=title, value=key, variable=self.source,
                            command=self.redraw).pack(side="left", padx=2)
        self.graph = tk.Canvas(self, height=153, bg="white", highlightthickness=0)
        self.graph.pack(fill="x")
        self.trail = tk.Label(self, bg="white", fg=INK, anchor="w", justify="left",
                              font=(FONT, 9))
        self.trail.pack(fill="x", padx=7)
        self.graph.bind("<Configure>", lambda event: self.redraw())

    def redraw(self):
        source = self.source.get()
        path = tuple(self.paths[source])
        width = self.graph.winfo_width()
        view = (source, path, width)
        if not path or width < 10 or view == self.last_view:
            return
        self.last_view = view
        c = self.graph
        c.delete("all")
        cell = width / 4
        half_w = min(81, cell / 2 - 9)
        half_h = 17
        centers = {name: ((col + 0.5) * cell, 25 + row * 50)
                   for row, names in enumerate(self.rows) for col, name in enumerate(names)}
        if len(path) > 1:
            x1, y1 = centers[path[-2]]
            x2, y2 = centers[path[-1]]
            if y1 == y2:
                points = (x1, y1-half_h, x1, y1-23, x2, y2-23, x2, y2-half_h)
            else:
                direction = 1 if y2 > y1 else -1
                lane_x = x1 + (1 if x2 >= x1 else -1) * (half_w+5)
                lane_y = y2-direction*25
                points = (x1, y1+direction*half_h, x1, y1+direction*22,
                          lane_x, y1+direction*22, lane_x, lane_y, x2, lane_y,
                          x2, y2-direction*half_h)
            c.create_line(*points, fill=GROUND, width=2, arrow="last",
                          arrowshape=(6, 7, 3), tags="latest_edge")
        for name, (x, y) in centers.items():
            active = name == path[-1]
            fill = GROUND if active else "#dcefe8" if name in path else "#f2f5f3"
            text_color = "white" if active else INK
            c.create_rectangle(x-half_w, y-half_h, x+half_w, y+half_h, fill=fill,
                               outline=GROUND if active else LINE, tags=("node", name))
            title = PHASES[name] if name != "POINT" else "指向 / 等待"
            c.create_text(x, y-7, text=title, fill=text_color, font=(FONT, 9))
            c.create_text(x, y+8, text=name, fill=text_color, font=("Consolas", 9))
        self.trail.configure(text="最近路径　" + " → ".join(path), wraplength=width-16)


class Workbench:
    def __init__(self, root):
        self.root = root
        self.sim = Simulation()
        self.playing = True
        self.drag = None
        self.pending = set()
        self.entries, self.values = {}, {}
        self.updating = False
        self.last_wall = time.perf_counter()
        self.credit = 0.0
        self.job = None
        self.event_key = None
        self.speed_dialog = None
        self.threshold_dialog = None
        self.io_dialog = None
        self.flight_options=("2","3","15","42","90","20","120")
        self.random_course=True
        self.wait_lock=True
        self.drive_options=("2","4","15","73","90","0.25","0.08","12","6","400")
        self.drive_random=True
        self.drive_wait=True
        self.auto_fit = True
        root.title("双端天线 · 交互闭环仿真")
        w = min(1320, root.winfo_screenwidth()-45)
        h = min(870, root.winfo_screenheight()-85)
        root.geometry(f"{w}x{h}")
        root.minsize(820, 580)
        root.configure(bg="#f4f6f2")
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure(".", font=(FONT,10))
        style.configure("TButton", padding=(10,5))
        style.configure("TEntry", padding=(4,2))
        header = tk.Frame(root, bg="#173936", padx=17, pady=8)
        header.pack(fill="x")
        tk.Label(header,text="双端天线  /  交互仿真",bg="#173936",fg="white",
                 font=(FONT,15,"bold")).pack(side="left")
        self.clock = tk.Label(header,bg="#173936",fg="#c6e5db",font=("Consolas",13))
        self.clock.pack(side="right", padx=5)
        ttk.Button(header, text="伺服速度配置", command=self.configure_speeds).pack(side="right", padx=12)
        ttk.Button(header, text="门限配置", command=self.configure_thresholds).pack(side="right", padx=2)
        self.realign_btn=ttk.Button(header,text="重新几何对准",command=self.request_realign)
        self.realign_btn.pack(side="right",padx=2)
        ttk.Button(header, text="算法输入输出", command=self.show_algorithm_io).pack(side="right", padx=2)
        main = tk.PanedWindow(root, orient="horizontal", sashwidth=5, bg=LINE, bd=0)
        main.pack(fill="both", expand=True)
        sidebar = tk.Frame(main, bg="white")
        main.add(sidebar, minsize=310, width=375)
        right = tk.Frame(main, bg="#f4f6f2")
        main.add(right, minsize=420, stretch="always")
        scroll = tk.Canvas(sidebar, bg="white", bd=0, highlightthickness=0)
        bar = ttk.Scrollbar(sidebar,orient="vertical",command=scroll.yview)
        bar.pack(side="right",fill="y")
        scroll.pack(side="left",fill="both",expand=True)
        scroll.configure(yscrollcommand=bar.set)
        self.side = tk.Frame(scroll,bg="white",padx=13,pady=8)
        inner = scroll.create_window(0,0,window=self.side,anchor="nw")
        self.side.bind("<Configure>",lambda e:scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<Configure>",lambda e:scroll.itemconfigure(inner,width=e.width))
        root.bind("<MouseWheel>",lambda e:scroll.yview_scroll(-int(e.delta/120),"units")
                  if str(e.widget).startswith(str(sidebar)) else None)
        self.phase = self.label(self.side, "",size=14,bold=True)
        self.status = self.label(self.side, "",wrap=285)
        self.reason = self.label(self.side, "",wrap=285)
        self.quality = self.label(self.side, "", bold=True, wrap=305)
        self.timer = self.label(self.side, "", wrap=310)
        self.history_count = 0
        self.targets = []
        for i in range(2):
            self.device_form(i)
        self.label(self.side, "重锁记录　失锁 → 锁定（仿真秒）", size=11, bold=True)
        history_frame = tk.Frame(self.side)
        history_frame.pack(fill="x")
        self.history = tk.Listbox(history_frame, height=3, font=("Consolas", 10))
        history_bar = ttk.Scrollbar(history_frame, command=self.history.yview)
        self.history.configure(yscrollcommand=history_bar.set)
        history_bar.pack(side="right", fill="y")
        self.history.pack(fill="x", expand=True)
        self.label(self.side,"状态跳转记录",size=11,bold=True)
        self.log = tk.Text(self.side,height=5,wrap="word",font=(FONT,9),fg=MUTED,
                           bg="#f3f6f4",relief="flat",padx=5,pady=4,state="disabled")
        self.log.pack(fill="x",pady=3)
        controls = tk.Frame(right,bg="#f4f6f2",padx=8,pady=6)
        controls.pack(fill="x")
        self.play_btn = ttk.Button(controls,text="暂停",width=5,command=self.toggle)
        self.play_btn.pack(side="left")
        ttk.Button(controls,text="重置",width=4,command=self.reset).pack(side="left",padx=2)
        self.speed = tk.StringVar(value="1")
        tk.Label(controls,text="播放倍率",bg="#f4f6f2",fg=INK).pack(side="left",padx=(8,2))
        ttk.Combobox(controls,width=3,state="readonly",values=("1","4","16","64"),
                     textvariable=self.speed).pack(side="left")
        ttk.Button(controls,text="适配视野",command=self.fit).pack(side="left",padx=5)
        self.mute = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls,text="遮挡信标",variable=self.mute,
                        command=self.toggle_beacon).pack(side="left")
        tracking_bar = ttk.Frame(right)
        tracking_bar.pack(fill="x", padx=8, pady=(0, 4))
        ttk.Button(tracking_bar, text="清空统计", command=self.clear_tracking_stats).pack(side="right")
        self.tracking_status = ttk.Label(tracking_bar)
        self.tracking_status.pack(side="left", fill="x", expand=True)
        flight_bar=ttk.Frame(right)
        flight_bar.pack(fill="x",padx=8)
        ttk.Button(flight_bar,text="连续飞行测试",command=self.configure_flight).pack(side="left")
        self.stop_flight_btn=ttk.Button(flight_bar,text="停止飞行",command=self.stop_flight)
        self.stop_flight_btn.pack(side="left",padx=4)
        self.flight_status=ttk.Label(flight_bar,wraplength=480)
        self.flight_status.pack(side="left",fill="x",expand=True)
        self.flight_status.bind("<Configure>",lambda e:self.flight_status.configure(wraplength=max(60,e.width-4)))
        drive_bar=ttk.Frame(right)
        drive_bar.pack(fill="x",padx=8)
        ttk.Button(drive_bar,text="车辆连续测试",command=lambda:self.configure_flight(True)).pack(side="left")
        self.stop_drive_btn=ttk.Button(drive_bar,text="停止行车",command=lambda:self.stop_flight(True))
        self.stop_drive_btn.pack(side="left",padx=4)
        self.drive_status=ttk.Label(drive_bar,wraplength=480)
        self.drive_status.pack(side="left",fill="x",expand=True)
        self.drive_status.bind("<Configure>",lambda e:self.drive_status.configure(wraplength=max(60,e.width-4)))
        self.canvas = tk.Canvas(right,bg="#eff4f0",highlightthickness=0)
        self.diagram = StateDiagram(right, self.sim.state_paths)
        self.diagram.pack(side="bottom", fill="x", padx=8, pady=4)
        guide = tk.Label(right, text="滚轮缩放 · 拖设备平移，拖圆柄或右键拖动旋转 · 侧视图上下拖动改变高度",
                         bg="#f4f6f2", fg=MUTED, font=(FONT, 9), wraplength=530)
        guide.pack(side="bottom", fill="x")
        self.canvas.pack(fill="both",expand=True,padx=8)
        self.canvas.bind("<Button-1>",lambda e:self.press(e,False))
        self.canvas.bind("<Button-3>",lambda e:self.press(e,True))
        self.canvas.bind("<MouseWheel>", self.zoom)
        self.canvas.bind("<Button-4>", lambda e: self.zoom(e, 1))
        self.canvas.bind("<Button-5>", lambda e: self.zoom(e, -1))
        self.canvas.bind("<Configure>", lambda event: self.fit() if self.auto_fit else self.draw())
        for button in (1,3):
            self.canvas.bind(f"<B{button}-Motion>",self.motion)
            self.canvas.bind(f"<ButtonRelease-{button}>",self.release)
        root.update_idletasks()
        self.fit()
        self.refresh()
        self.last_wall = time.perf_counter()
        root.protocol("WM_DELETE_WINDOW",self.close)
        self.job = root.after(20,self.tick)

    @staticmethod
    def label(parent,text,size=10,bold=False,bg="white",wrap=0):
        widget = tk.Label(parent,text=text,bg=bg,fg=INK,anchor="w",justify="left",
                          font=(FONT,size,"bold" if bold else "normal"),wraplength=wrap)
        widget.pack(fill="x",pady=2)
        return widget

    def device_form(self,i):
        form=tk.Frame(self.side,bg="white")
        form.pack(fill="x",pady=(7,2))
        tk.Label(form,text=NAMES[i],font=(FONT,12,"bold"),fg=(AIR,GROUND)[i],
                 bg="white").grid(row=0,column=0,columnspan=2,sticky="w")
        ttk.Button(form,text="应用",width=4,command=lambda:self.apply(i)).grid(row=0,column=3)
        for j,(key,name) in enumerate(FIELDS):
            row,col=1+j//2,(j%2)*2
            if key=="el":
                name="实测俯角 °" if i==0 else "实测仰角 °"
            tk.Label(form,text=name,bg="white",fg=MUTED,font=(FONT,9)).grid(row=row,column=col,sticky="w")
            var=tk.StringVar()
            entry=ttk.Entry(form,width=10,textvariable=var,font=("Consolas",10))
            entry.grid(row=row,column=col+1,sticky="ew",pady=2,padx=(2,4))
            form.grid_columnconfigure(col+1,weight=1)
            self.entries[i,key]=entry
            self.values[i,key]=var
            var.trace_add("write",lambda *_, ix=i,k=key:self.mark(ix,k))
            entry.bind("<Return>",lambda e,ix=i:self.apply(ix))
        self.targets.append(self.label(self.side,"",size=9))

    def mark(self,i,k):
        if not self.updating:
            self.pending.add((i,k))

    @property
    def poses(self):
        return (self.sim.air,self.sim.ground)

    def apply(self,i):
        edits={}
        for key,title in FIELDS:
            if (i,key) not in self.pending:
                continue
            try:
                x=float(self.values[i,key].get())
                if not math.isfinite(x): raise ValueError
                if key in ("lat","lon","el"):
                    low,high={"lat":(-90,90),"lon":(-180,180),"el":(0,90)}[key]
                    if not low<=x<=high: raise ValueError
                if key=="height" and not -12000<=x<=100000: raise ValueError
            except ValueError:
                hint="俯仰角范围为 0° 到 90°。" if key=="el" else f"请检查「{title}」的数值与范围。"
                messagebox.showerror("输入无效",hint,parent=self.root)
                return
            edits[key]=x%360 if key in ("az","heading") else x
        if "height" in edits and edits["height"]!=self.sim.bounded_height(self.poses[i],edits["height"]):
            messagebox.showerror("高度无效",f"机端海拔至少要比车端高 {MIN_HEIGHT_GAP_M:g} m。",parent=self.root)
            return
        for key,x in edits.items():
            obj=self.sim.axes[i] if key in ("az","el") else self.poses[i]
            setattr(obj,key,x)
            self.pending.discard((i,key))
        self.canvas.focus_set()
        self.sim.measure_link()
        if edits.keys() & {"lat", "lon", "height"}: self.fit()
        self.refresh()

    def toggle(self):
        self.playing=not self.playing
        self.last_wall=time.perf_counter()
        self.play_btn.configure(text="暂停" if self.playing else "继续")

    def toggle_beacon(self):
        self.sim.blocked=self.mute.get()
        self.sim.measure_link()
        self.refresh()

    def request_realign(self):
        self.sim.realign_pending=True
        self.refresh()

    def reset(self):
        fresh = Simulation()
        if self.sim.max_slant>fresh.distance:
            fresh.max_slant=self.sim.max_slant
        fresh.set_thresholds(self.sim.fixed_thresholds)
        fresh.realign_pending=False  # already at the initial geometric point
        for new, old in zip(fresh.axes, self.sim.axes):
            new.az_speed, new.el_speed = old.az_speed, old.el_speed
        self.sim.close()
        self.sim = fresh
        self.diagram.paths = fresh.state_paths
        self.diagram.last_view = None
        self.event_key = None
        self.history.delete(0, "end")
        self.history_count = 0
        self.drag = None
        self.pending.clear()
        self.canvas.focus_set()
        self.mute.set(False)
        self.credit = 0
        self.last_wall = time.perf_counter()
        self.fit()
        self.refresh()

    def show_algorithm_io(self):
        if self.io_dialog is not None and self.io_dialog.winfo_exists():
            self.io_dialog.deiconify()
            self.io_dialog.lift()
            return
        window = self.io_dialog = tk.Toplevel(self.root)
        window.title("算法输入输出")
        window.geometry(f"{min(760, self.root.winfo_screenwidth()-60)}x{min(680, self.root.winfo_screenheight()-100)}")
        bar = ttk.Frame(window, padding=8)
        bar.pack(fill="x")
        self.io_device = tk.StringVar(window, value="ground")
        self.io_call = tk.StringVar(window, value="step")
        for variable, choices in ((self.io_device, (("机载", "air"), ("车载", "ground"))),
                                  (self.io_call, (("初始化", "init"), ("跟踪调用（10 ms）", "step")))):
            group = ttk.Frame(bar)
            group.pack(side="left", padx=(0, 16))
            for label, key in choices:
                ttk.Radiobutton(group, text=label, variable=variable, value=key,
                                command=self.refresh_algorithm_io).pack(side="left")
        self.io_text = ScrolledText(window, wrap="word", bg="white", fg=INK,
                                    font=(FONT, 10), padx=12, pady=8, state="disabled")
        self.io_text.pack(fill="both", expand=True)
        self.io_view = None
        self.refresh_algorithm_io()

    def refresh_algorithm_io(self):
        if self.io_dialog is None or not self.io_dialog.winfo_exists():
            return
        view = (self.sim, self.io_device.get(), self.io_call.get())
        changed = view != self.io_view
        initialization = self.io_call.get() == "init"
        now = time.monotonic()
        if not changed and (initialization or self.io_tick == self.sim.ms or now < self.io_next_refresh):
            return
        snapshot = self.sim.call_snapshot()
        content = call_text(snapshot, self.io_device.get() == "air", initialization, MODES)
        position = 0.0 if changed else self.io_text.yview()[0]
        self.io_text.configure(state="normal")
        self.io_text.delete("1.0", "end")
        self.io_text.insert("1.0", content)
        self.io_text.yview_moveto(position)
        self.io_text.configure(state="disabled")
        self.io_view, self.io_tick = view, self.sim.ms
        self.io_next_refresh = now + 0.15

    def clear_tracking_stats(self):
        self.sim.clear_tracking_stats()
        self.refresh()

    def stop_flight(self, ground=False):
        run=self.sim.drive if ground else self.sim.flight
        if run:run.active=False
        self.refresh()

    def configure_flight(self, ground=False):
        d=tk.Toplevel(self.root)
        d.title("车辆连续测试" if ground else "无人机连续飞行测试")
        d.transient(self.root)
        form=ttk.Frame(d,padding=14)
        form.pack()
        random_course=tk.BooleanVar(value=self.drive_random if ground else self.random_course)
        wait_lock=tk.BooleanVar(value=self.drive_wait if ground else self.wait_lock)
        entries=[]
        labels=["最低速度 (m/s)","最高速度 (m/s)","每段时长 (s)","随机种子","固定行进方向 (°，北为 0)"]
        if ground:
            labels.extend(["最大转向速度 (°/s)","颠簸幅值 (m)","颠簸基准波长 (m)",
                           "坡道高度幅值 (m)","坡道波长 (m)"])
        else:
            labels.extend(["相对起始高度幅值 (±m)","升降周期 (s)"])
        labels.append("两站共用斜距上限 (m)")
        options=self.drive_options if ground else self.flight_options
        for row,(label,value) in enumerate(zip(labels,(*options,f"{self.sim.max_slant:g}"))):
            ttk.Label(form,text=label).grid(row=row,column=0,sticky="w")
            entry=ttk.Entry(form,width=12)
            entry.insert(0,value)
            entry.grid(row=row,column=1,pady=2)
            entries.append(entry)
        def update_course():
            entries[4].configure(state="disabled" if random_course.get() else "normal")
        row=len(labels)
        ttk.Checkbutton(form,text="每段随机方向",variable=random_course,command=update_course).grid(row=row,columnspan=2,sticky="w")
        update_course()
        ttk.Checkbutton(form,text="先锁定，再开始移动",variable=wait_lock).grid(row=row+1,columnspan=2,sticky="w")
        hint=("坡道随行驶里程改变海拔，可与短波颠簸叠加。"
              if ground else "高度围绕起始海拔平滑升降，幅值填 0 可等高飞行。")
        ttk.Label(form,text=hint,wraplength=300).grid(row=row+2,columnspan=2,pady=4)
        error=ttk.Label(form,foreground="#aa4227",wraplength=300)
        error.grid(row=row+3,columnspan=2)
        def start():
            values=tuple(entry.get() for entry in entries[:-1])
            try:
                low,high,interval=map(float,values[:3])
                course=None if random_course.get() else float(values[4])
                args=(low,high,course,interval,int(values[3]),wait_lock.get())
                run=(DriveTest if ground else FlightTest)(*args,*map(float,values[5:]))
                limit=float(entries[-1].get())
                if self.sim.motion_center is None or limit!=self.sim.max_slant:
                    self.sim.reset_motion_area(limit)
            except ValueError as exc:
                error.configure(text=f"请检查输入（随机种子为整数）。{exc}")
                return
            if ground:
                self.drive_options=values
                self.drive_random,self.drive_wait=random_course.get(),wait_lock.get()
                self.sim.drive=run
            else:
                self.flight_options=values
                self.random_course,self.wait_lock=random_course.get(),wait_lock.get()
                self.sim.flight=run
            d.destroy()
            self.refresh()
        ttk.Button(form,text="取消",command=d.destroy).grid(row=row+4,column=0)
        ttk.Button(form,text="开始新测试",command=start).grid(row=row+4,column=1)
        d.grab_set()

    def configure_thresholds(self):
        if self.threshold_dialog and self.threshold_dialog.winfo_exists():
            self.threshold_dialog.lift()
            return
        dialog=tk.Toplevel(self.root)
        self.threshold_dialog=dialog
        dialog.title("信标门限配置")
        dialog.transient(self.root)
        form=ttk.Frame(dialog,padding=16)
        form.pack()
        s=self.sim
        mode=tk.StringVar(value="fixed" if s.fixed_thresholds is not None else "auto")
        entries=[]
        def update_mode():
            for entry in entries:
                entry.configure(state="normal" if mode.get()=="fixed" else "disabled")
        for col,(value,title) in enumerate((("auto","按距离自动"),("fixed","固定门限"))):
            ttk.Radiobutton(form,text=title,value=value,variable=mode,command=update_mode).grid(row=0,column=col)
        levels=s.fixed_thresholds or (-116,s.out.severe,s.out.release,s.out.acquire)
        for row,(label,value) in enumerate(zip(("检出","极弱重对准","保持","锁定"),levels),1):
            ttk.Label(form,text=label).grid(row=row,column=0,sticky="w")
            entry=ttk.Entry(form,width=12)
            entry.insert(0,str(value) if s.fixed_thresholds is not None else f"{value:.2f}")
            entry.grid(row=row,column=1,pady=3)
            entries.append(entry)
        update_mode()
        ttk.Label(form,text="更改后，下一控制拍重新对准并确认锁定。").grid(row=5,columnspan=2,pady=5)
        error=ttk.Label(form,foreground="#aa4227",wraplength=290)
        error.grid(row=6,columnspan=2)
        def apply(event=None):
            try:
                values=tuple(float(entry.get()) for entry in entries) if mode.get()=="fixed" else None
            except ValueError:
                error.configure(text="请输入 −120～0 范围内的数值。")
                return
            try:
                self.sim.set_thresholds(values)
            except ValueError as exc:
                error.configure(text=str(exc))
                return
            dialog.destroy()
            self.refresh()
        ttk.Button(form,text="取消",command=dialog.destroy).grid(row=7,column=0)
        ttk.Button(form,text="应用",command=apply).grid(row=7,column=1)
        dialog.bind("<Return>",apply)
        dialog.bind("<Escape>",lambda event: dialog.destroy())
        dialog.grab_set()

    def configure_speeds(self):
        if self.speed_dialog and self.speed_dialog.winfo_exists():
            self.speed_dialog.lift()
            return
        dialog = tk.Toplevel(self.root)
        self.speed_dialog = dialog
        dialog.title("伺服速度配置")
        dialog.transient(self.root)
        dialog.resizable(False, False)
        form = ttk.Frame(dialog, padding=14)
        form.pack()
        entries = []
        for col, label in enumerate(("设备", "方位 (°/s)", "俯仰 (°/s)")):
            ttk.Label(form, text=label).grid(row=0, column=col, padx=5)
        for row, (name, axes) in enumerate(zip(NAMES, self.sim.axes), 1):
            ttk.Label(form, text=name).grid(row=row, column=0, padx=5)
            for col, attr in enumerate(("az_speed", "el_speed"), 1):
                entry = ttk.Entry(form, width=10)
                entry.insert(0, str(getattr(axes, attr)))
                entry.grid(row=row, column=col, padx=5, pady=4)
                entries.append((entry, axes, attr))
        error = ttk.Label(form, foreground="#aa4227")
        error.grid(row=3, column=0, columnspan=3, sticky="w")

        def apply_rates(event=None):
            updates = []
            for entry, axes, attr in entries:
                try:
                    rate = float(entry.get())
                    if not math.isfinite(rate) or rate < 0:
                        raise ValueError
                except ValueError:
                    error.configure(text="请输入大于或等于 0 的有限数值。")
                    entry.focus_set()
                    return
                updates.append((axes, attr, rate))
            for axes, attr, rate in updates:
                setattr(axes, attr, rate)
            dialog.destroy()
            self.refresh()

        ttk.Button(form, text="取消", command=dialog.destroy).grid(row=4, column=1, pady=3)
        ttk.Button(form, text="应用", command=apply_rates).grid(row=4, column=2, pady=3)
        dialog.bind("<Return>", apply_rates)
        dialog.bind("<Escape>", lambda event: dialog.destroy())
        dialog.grab_set()
        entries[0][0].focus_set()

    def fit(self):
        self.auto_fit = True
        a,g=self.poses
        self.lat0=(a.lat+g.lat)/2
        self.lon0=wrap_diff(g.lon+wrap_diff(a.lon-g.lon)/2)
        self.metres_lon=111320*max(0.001,math.cos(math.radians(self.lat0)))
        span_x=abs(wrap_diff(a.lon-g.lon)*self.metres_lon)
        span_y=abs(a.lat-g.lat)*111320
        self.span=max(300,span_x*1.8,span_y*1.8*self.map_width()/self.map_height())
        self.max_span = max(40075000, self.span)
        self.zmin=min(a.height,g.height)-max(15,abs(a.height-g.height)*0.3)
        self.zspan=max(80,(max(a.height,g.height)-self.zmin)*1.35)
        self.draw()

    def map_width(self):return max(1,self.canvas.winfo_width())
    def map_height(self):return max(1,self.canvas.winfo_height()*0.68)

    def zoom(self, event, steps=None):
        w, h = self.map_width(), self.map_height()
        if not 0 <= event.y < h or self.drag is not None:
            return
        if steps is None:
            steps = event.delta / 120 if sys.platform == "win32" else event.delta
        self.auto_fit = False
        factor = 1.25 ** -max(-12, min(12, steps))
        span = max(2, min(self.max_span, self.span * factor))
        # Shift the map center to keep the geographic point under the cursor fixed.
        delta = (self.span - span) / w
        self.lon0 = wrap_diff(self.lon0 + (event.x-w/2)*delta/self.metres_lon)
        self.lat0 += (h/2-event.y)*delta/111320
        self.span = span
        self.draw()
        return "break"

    def xy(self,p):
        w,h=self.map_width(),self.map_height()
        return (w/2+wrap_diff(p.lon-self.lon0)*self.metres_lon*w/self.span,
                h/2-(p.lat-self.lat0)*111320*w/self.span)

    def profile_y(self,height):
        h=self.canvas.winfo_height()
        return h-26-(height-self.zmin)*(h-self.map_height()-57)/self.zspan

    def press(self,e,right):
        self.canvas.focus_set()
        for i in (0,1):
            x,y=self.xy(self.poses[i])
            kx,ky=self.handle_xy(i)
            px,py=self.profile_xy(i)
            if e.y>self.map_height() and math.hypot(e.x-px,e.y-py)<21:
                self.drag=("height",i,py-e.y,0)
                return
            if e.y>self.map_height(): continue
            if math.hypot(e.x-kx,e.y-ky)<14 or (right and math.hypot(e.x-x,e.y-y)<28):
                self.drag=("heading",i,0,0)
                return
            if not right and math.hypot(e.x-x,e.y-y)<24:
                self.drag=("position",i,x-e.x,y-e.y)
                return

    def motion(self,e):
        if self.drag is None:return
        kind,i,dx,dy=self.drag
        p=self.poses[i]
        w,h=self.map_width(),self.map_height()
        if kind=="position":
            px=max(18,min(w-18,e.x+dx))
            py=max(24,min(h-18,e.y+dy))
            p.lat=max(-90,min(90,self.lat0+(h/2-py)*self.span/w/111320))
            p.lon=wrap_diff(self.lon0+(px-w/2)*self.span/w/self.metres_lon)
            keys=("lat","lon")
        elif kind=="heading":
            x,y=self.xy(p)
            if math.hypot(e.x-x,e.y-y)<4:return
            p.heading=math.degrees(math.atan2(e.x-x,y-e.y))%360
            keys=("heading",)
        else:
            bottom=self.canvas.winfo_height()-26
            py=max(h+33,min(bottom,e.y+dx))
            height=self.zmin+(bottom-py)*self.zspan/max(1,bottom-h-31)
            p.height=self.sim.bounded_height(p,height)
            keys=("height",)
        for key in keys:self.pending.discard((i,key))
        self.sim.measure_link()
        self.refresh()

    def release(self,e):
        self.drag=None

    def handle_xy(self,i):
        x,y=self.xy(self.poses[i])
        t=math.radians(self.poses[i].heading)
        return x+42*math.sin(t),y-42*math.cos(t)

    def profile_xy(self,i):
        return self.map_width()*(0.75 if i==0 else 0.25),self.profile_y(self.poses[i].height)

    def draw(self):
        c=self.canvas
        c.delete("all")
        w,h=self.map_width(),self.map_height()
        height=c.winfo_height()
        if w<2 or height<2:return
        def note(x,y,s,anchor="nw",color=MUTED,size=10):
            c.create_text(x,y,text=s,anchor=anchor,fill=color,font=(FONT,size))
        spacing=10**math.floor(math.log10(self.span/7))
        if self.span/spacing>16:spacing*=5
        elif self.span/spacing>8:spacing*=2
        gap=w*spacing/self.span
        for origin,limit,vertical in ((w/2,w,True),(h/2,h,False)):
            v=origin%gap
            while v<limit:
                if vertical:c.create_line(v,0,v,h,fill="#dce6df")
                else:c.create_line(0,v,w,v,fill="#dce6df")
                v+=gap
        a,g=map(self.xy,self.poses)
        for run,color in ((self.sim.flight,AIR),(self.sim.drive,GROUND)):
            if run and len(run.trail)>1:
                c.create_line([self.xy(p) for p in run.trail],fill=color,dash=(3,3))
        for i,p in enumerate(self.poses):
            x,y=self.xy(p)
            color=(AIR,GROUND)[i]
            t,el=map(math.radians,beam_direction(p,self.sim.axes[i]))
            radius=self.sim.distance*w/self.span
            points=beam_footprint(x,y,radius,t,el)
            if len(points)>=3:
                c.create_polygon(points,fill=color,stipple="gray12",outline=color)
            length=radius*math.cos(el)
            c.create_line(x,y,x+length*math.sin(t),y-length*math.cos(t),fill=color,width=2)
        c.create_line(*a,*g,dash=(5,5),fill="#526768")
        for i,p in enumerate(self.poses):
            x,y=self.xy(p)
            color=(AIR,GROUND)[i]
            kx,ky=self.handle_xy(i)
            c.create_line(x,y,kx,ky,fill=color,width=2,dash=(3,3))
            c.create_oval(kx-5,ky-5,kx+5,ky+5,fill="white",outline=color,width=3)
            t=math.radians(p.heading)
            def point(right,forward):
                return x+right*math.cos(t)+forward*math.sin(t), y+right*math.sin(t)-forward*math.cos(t)
            if i==0:
                for lateral in (-1,1):
                    for forward in (-1,1):
                        xx,yy=point(11*lateral,10*forward)
                        c.create_line(x,y,xx,yy,fill=color,width=3)
                        c.create_oval(xx-5,yy-5,xx+5,yy+5,outline=color,width=2,fill="#faede1")
                c.create_polygon([point(0,11),point(7,-6),point(-7,-6)],fill=color)
            else:
                c.create_polygon([point(-9,-15),point(9,-15),point(9,10),point(5,16),point(-7,16)],fill=color)
                c.create_line(*point(-6,6),*point(6,6),fill="#d2eee5",width=3)
            note(x,y+23,NAMES[i],"n",color)
        note(12,8,"主瓣水平投影　·　按两站斜距绘制，圆柄为平台航向")
        note(w-12,8,"↑ 北","ne")
        c.create_line(13,h-16,13+gap,h-16,fill=MUTED,width=2)
        note(13,h-19,f"{spacing:g} m","sw",size=9)
        c.create_rectangle(0,h,w,height,fill="#f8faf6",outline=LINE)
        note(12,h+7,"高度剖面　·　横向为两站连线，箭头为天线轴投影")
        profile=list(map(self.profile_xy,(0,1)))
        c.create_line(*profile[0],*profile[1],dash=(4,4),fill="#8b9c98")
        # Profile uses separate horizontal and vertical scales.
        horizontal=max(1,math.hypot(wrap_diff(self.sim.air.lon-self.sim.ground.lon)*self.metres_lon,
                                      (self.sim.air.lat-self.sim.ground.lat)*111320))
        for i,(px,py) in enumerate(profile):
            j=1-i; x,y=self.xy(self.poses[i]); ox,oy=self.xy(self.poses[j])
            toward=math.atan2(ox-x,y-oy)
            axis=self.sim.axes[i]
            az,el=map(math.radians,beam_direction(self.poses[i],axis))
            vx=(1 if i else -1)*math.cos(az-toward)*math.cos(el)*w*0.5/horizontal
            vy=-math.sin(el)*(height-h-57)/self.zspan
            scale=42/max(math.hypot(vx,vy),1e-9)
            color=(AIR,GROUND)[i]
            c.create_line(px,py,px+vx*scale,py+vy*scale,fill=color,width=3,arrow="last")
            c.create_oval(px-7,py-7,px+7,py+7,fill=color,outline="white",width=2)
            note(px,py+9,f"{NAMES[i]}  {self.poses[i].height:.2f} m","n",color,9)

    def refresh(self):
        self.refresh_algorithm_io()
        s=self.sim
        o=s.out
        effective, elapsed = s.tracking_stats_seconds
        percentage = f"{effective / elapsed:.2%}" if elapsed else "—"
        self.tracking_status.configure(
            text=f"有效跟踪占比 {percentage}　有效 {effective:.2f} s / 总计 {elapsed:.2f} s")
        flight=s.flight
        self.stop_flight_btn.configure(state="normal" if flight and flight.active else "disabled")
        summary=""
        if flight:
            status="已停止" if not flight.active else "等待锁定" if flight.waiting else "边界折返" if flight.returning else "飞行中"
            summary=(f"{status}  {flight.ticks*DT:.1f} s　{flight.actual_speed if flight.active else 0:.1f} m/s　航迹 {flight.course:.0f}°"
                     f"　有效跟踪 {flight.track_ticks*DT:.1f} s　失锁 {flight.losses}　回几何 {flight.near_returns}")
            if MODES[o.phase]=="TRACK":
                summary+=f"\n车端峰心　方位 {o.center_az:.2f}°　仰角 {o.center_el:.2f}°"
        self.flight_status.configure(text=summary+(f"　{flight.stop_reason}" if flight and flight.stop_reason else ""))
        drive=s.drive
        self.stop_drive_btn.configure(state="normal" if drive and drive.active else "disabled")
        summary=""
        if drive:
            status="已停车" if not drive.active else "等待锁定" if drive.waiting else "边界折返" if drive.returning else "行车中"
            summary=(f"{status}  {drive.ticks*DT:.1f} s　{drive.actual_speed if drive.active else 0:.1f} m/s"
                     f"　车头 {s.ground.heading:.1f}°　纵倾 {s.axes[1].body_pitch:+.2f}°"
                     f"　侧倾 {s.axes[1].body_roll:+.2f}°　起伏 {drive.heave:+.2f} m"
                     f"\n有效跟踪 {drive.track_ticks*DT:.1f} s　失锁 {drive.losses}　回几何 {drive.near_returns}")
        self.drive_status.configure(text=summary+(f"　{drive.stop_reason}" if drive and drive.stop_reason else ""))
        if self.auto_fit and self.drag is None and any(r and r.active for r in (flight,drive)):
            if (any(not (15<x<self.map_width()-15 and 15<y<self.map_height()-15)
                    for x,y in map(self.xy,self.poses)) or
                any(not self.zmin<=p.height<=self.zmin+self.zspan for p in self.poses)):
                self.fit()
        self.updating=True
        focus_path=str(self.root.tk.call("focus"))
        for i,p in enumerate(self.poses):
            for key,_ in FIELDS:
                if (i,key) in self.pending or focus_path==str(self.entries[i,key]):continue
                obj=s.axes[i] if key in ("az","el") else p
                val=getattr(obj,key)
                txt=f"{val:.6f}" if key in ("lat","lon") else f"{val:.2f}"
                if self.values[i,key].get()!=txt:self.values[i,key].set(txt)
            target=(o.air,o.ground)[i]
            pitch_label="目标俯角 ↓" if i==0 else "目标仰角 ↑"
            target_text=(f"目标方位 {target.az:.2f}°   /   {pitch_label} {target.el:.2f}°"
                         if target.valid else "目标角无效 · 保持实际角")
            axes=s.axes[i]
            self.targets[i].configure(text=f"{target_text}\n限速　方位 {axes.az_speed:g}°/s   /   俯仰 {axes.el_speed:g}°/s")
        self.updating=False
        phase=MODES[o.phase]
        self.clock.configure(text=f"T + {(s.ms-10)/1000:07.2f} s")
        active=phase if o.ground.valid else MODES[o.ground.mode]
        self.phase.configure(text=f"{PHASES[active]}   {active}")
        action=("回峰复测" if o.confirm and phase=="CAL_AIR" else
                "车端小十字" if o.rx_probe else "到位采样" if o.settling else "云台转动")
        detail=(f"{action if o.ack else '等待机端匹配回执'}  /  点 {o.point+1}"
                if o.ground.valid else "等待有效输入")
        self.status.configure(text=f"机端 {MODES[o.air.mode]}  ·  车端 {MODES[o.ground.mode]}\n"
            f"{detail}  /  "
            f"信标 {'检出' if s.detected else '无'} {s.level:.1f}\n斜距 {s.distance:.1f} m　自动运动上限 {s.max_slant:g} m"
            f"\n机端高于车端 {s.air.height-s.ground.height:.2f} m")
        self.reason.configure(text="最近跳转　"+s.events[-1][3])
        self.realign_btn.configure(text="等待下一控制拍" if s.realign_pending else "重新几何对准",
                                   state="disabled" if s.realign_pending else "normal")
        if s.aligned_lock:
            lock_text = "已锁定 · 双端在主瓣内"
        elif not s.in_beams:
            lock_text = "未对准 · 至少一端偏离主瓣"
        elif not s.detected:
            lock_text = "主瓣对准 · 无有效信标"
        elif s.out.holding:
            lock_text = "主瓣对准 · 保持强度不足"
        else:
            lock_text = "主瓣对准 · 等待控制器确认"
        self.quality.configure(fg=GROUND if s.aligned_lock else "#974a20",
            text=f"{lock_text}\n离轴　机 {s.errors[0]:.2f}° / 车 {s.errors[1]:.2f}°（半角 {BEAM_HALF_DEG:g}°）"
                 f"\n{'固定' if s.fixed_thresholds is not None else '自动'} · 锁定 {o.acquire:.1f}　保持 {o.release:.1f}　极弱 {o.severe:.1f}"
                 + (f"\n上轮中心 {o.track_center:.2f}　五点最强 {o.track_best:.2f}"
                    if min(o.track_center,o.track_best)>=-120 else ""))
        if s.loss_start_ms is not None:
            timer_text=f"第 {len(s.relocks)+1} 次失锁　已用 {(s.ms-10-s.loss_start_ms)/1000:.2f} s"
        elif s.relocks:
            start,end=s.relocks[-1]
            timer_text=f"第 {len(s.relocks)} 次重锁　用时 {(end-start)/1000:.2f} s"
        elif s.first_lock_ms is not None:
            timer_text=f"首次锁定 {s.first_lock_ms/1000:.2f} s　尚无失锁记录"
        else:
            timer_text="首次捕获中　重锁计时待命"
        self.timer.configure(text=timer_text)
        while self.history_count<len(s.relocks):
            start,end=s.relocks[self.history_count]
            self.history_count+=1
            self.history.insert(0,f"#{self.history_count}  {start/1000:.2f} → {end/1000:.2f}    {(end-start)/1000:.2f} s")
        if self.event_key!=s.events[-1]:
            self.event_key=s.events[-1]
            self.log.configure(state="normal")
            self.log.delete("1.0","end")
            self.log.insert("end","\n".join(f"{t:6.2f}s  {phase} / {out}　{reason}"
                               for t,phase,out,reason in reversed(s.events[-12:])))
            self.log.configure(state="disabled")
        self.diagram.redraw()
        self.draw()

    def tick(self):
        wall=time.perf_counter()
        if self.playing:
            self.credit+=min(0.2,wall-self.last_wall)*int(self.speed.get())
            while self.credit>=DT:
                self.sim.step()
                self.credit-=DT
        self.last_wall=wall
        self.refresh()
        self.job=self.root.after(30,self.tick)

    def close(self):
        if self.job:self.root.after_cancel(self.job)
        self.sim.close()
        self.root.destroy()


if __name__=="__main__":
    root=tk.Tk()
    try:
        Workbench(root)
    except Exception as exc:
        messagebox.showerror("仿真启动失败",str(exc),parent=root)
        root.destroy()
        raise
    root.mainloop()

