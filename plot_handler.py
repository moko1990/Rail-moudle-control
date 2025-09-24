# filename: plot_handler.py
# Centralized plotting logic for Linear Rail Controller

import time
from collections import deque
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject
from PySide6.QtWidgets import QToolTip
from PySide6.QtGui import QCursor


class TimeAxisItem(pg.AxisItem):
    """A custom axis item that displays time in H:MM:SS, MM:SS, or S."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setLabel(text='Time', units='s')
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values, scale, spacing):
        out = []
        for sec in values:
            if sec < 0:
                out.append("-")
            elif sec < 60:
                out.append(f"{sec:.0f}")
            elif sec < 3600:
                m = int(sec // 60)
                s = int(sec % 60)
                out.append(f"{m:d}:{s:02d}")
            else:
                h = int(sec // 3600)
                rem = sec % 3600
                m = int(rem // 60)
                s = int(rem % 60)
                out.append(f"{h:d}:{m:02d}:{s:02d}")
        return out


class ResistanceAxisItem(pg.AxisItem):
    """Y axis that formats resistance values (stored in kΩ) into Ω/kΩ/MΩ nicely."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values, scale, spacing):
        out = []
        for r_k in values:
            try:
                r = float(r_k)
            except Exception:
                out.append("")
                continue
            if r >= 1000.0:
                out.append(f"{r/1000.0:.3g} MΩ")
            elif r >= 1.0:
                out.append(f"{r:.3g} kΩ")
            else:
                out.append(f"{(r*1000.0):.3g} Ω")
        return out


class PlotHandler(QObject):
    def __init__(self, ui, plot_time_window=30.0, max_time_points=10000, keep_last_n_cycles = 4):
        super().__init__()
        self.ui = ui
        self.plot_time_window = float(plot_time_window)
        self.max_time_points = int(max_time_points)
        self.keep_last_n_cycles = int(keep_last_n_cycles)
        self.max_legs = self.keep_last_n_cycles * 2

        # Smooth scroll params
        self.scroll_alpha = 0.25   # 0..1 هرچه بیشتر نرم‌تر (0.2..0.35 پیشنهاد)
        self._adc_xmax_smooth = 0.0
        self._res_xmax_smooth = 0.0

        # Time-series buffers
        self._t0 = time.time()
        self.adc_time = deque(maxlen=self.max_time_points)
        self.adc_volt = deque(maxlen=self.max_time_points)
        self.res_time = deque(maxlen=self.max_time_points)
        self.res_val = deque(maxlen=self.max_time_points)

        # Displacement legs
        self.vd_legs = []   # list of list[(dist_mm, volt)]
        self.rd_legs = []   # list of list[(dist_mm, res_kohm)]
        self.vd_curves = []
        self.rd_curves = []
        self.vd_leg_colors = []  # هم‌اندازه vd_legs: اندیس رنگ 0..3
        self.rd_leg_colors = []
        self.leg_max_dist_mm = 0.0

        # Palettes
        # جایگزین هر دو پالت با یک پالت مشترک
        self.cycle_pens = [
            pg.mkPen(color=(0, 180, 0), width=1),     # سبز
            pg.mkPen(color=(220, 30, 30), width=1),   # قرمز
            pg.mkPen(color=(0, 150, 255), width=1),   # آبی
            pg.mkPen(color=(255, 150, 0), width=1)    # نارنجی
        ]

        # Time curves
        pen_adc = pg.mkPen(color=(220, 30, 30), width=1)
        pen_res = pg.mkPen(color=(0, 150, 255), width=1)
        self.adc_curve = self.ui.adc_plot_widget.plot(pen=pen_adc, name="ADC Voltage")
        self._config_curve(self.adc_curve)  # A) apply on curve
        self.res_curve = self.ui.res_plot_widget.plot(pen=pen_res, name="Resistance")
        self._config_curve(self.res_curve)  # A) apply on curve

        # Hover cache
        self._adc_t_arr = None
        self._adc_v_arr = None
        self._res_t_arr = None
        self._res_r_arr = None

        # Hover throttle
        self._last_hover_ts = 0.0
        self._hover_min_period = 0.03  # seconds

        # Configure plots
        self._setup_plots()

    # ============ A) helper to configure curves ============
    def _config_curve(self, curve):
        """Ensure downsampling/clip are applied on the actual curve objects."""
        try:
            curve.setClipToView(True)
            curve.setDownsampling(mode='peak', auto=True)
        except Exception:
            pass

    def _setup_plots(self):
        # ADC plot
        adc_w = self.ui.adc_plot_widget
        adc_w.setBackground('w')
        pa = adc_w.getPlotItem()
        pa.showGrid(x=True, y=True, alpha=0.3)
        pa.setLabel('left', 'Voltage', units='V')
        pa.setAxisItems({'bottom': TimeAxisItem(orientation='bottom')})
        pa.setClipToView(True)
        pa.setDownsampling(mode='peak', auto=True)
        pa.getViewBox().setLimits(xMin=0, yMin=-0.2, yMax=4.5, minYRange=0.01)

        # Resistance plot
        res_w = self.ui.res_plot_widget
        res_w.setBackground('w')
        pr = res_w.getPlotItem()
        pr.setLabel('left', 'Resistance')
        pr.setAxisItems({'bottom': TimeAxisItem(orientation='bottom'),
                         'left': ResistanceAxisItem(orientation='left')})
        # D) showGrid on PlotItem is sufficient; remove numeric axis.setGrid calls
        pr.showGrid(x=False, y=True, alpha=0.3)
        pr.setClipToView(True)
        pr.setDownsampling(mode='peak', auto=True)
        pr.setYRange(1.0, 200.0, padding=0)
        pr.getViewBox().setLimits(xMin=0, yMin=0, yMax=500.0, minYRange=0.1, maxYRange=500.0)

        # V–D plot
        if hasattr(self.ui, 'v_vs_d_plot_widget') and self.ui.v_vs_d_plot_widget:
            vd_w = self.ui.v_vs_d_plot_widget
            vd_w.setBackground('w')
            pv = vd_w.getPlotItem()
            pv.showGrid(x=True, y=True, alpha=0.3)
            pv.setLabel('left', 'Voltage', units='V')
            pv.setLabel('bottom', 'Relative Displacement', units='mm')
            pv.setClipToView(True)
            pv.setDownsampling(mode='peak', auto=True)
            pv.getViewBox().setLimits(xMin=0, yMin=-0.2, yMax=4.5, minYRange=0.01)

        # R–D plot
        if hasattr(self.ui, 'r_vs_d_plot_widget') and self.ui.r_vs_d_plot_widget:
            rd_w = self.ui.r_vs_d_plot_widget
            rd_w.setBackground('w')
            prd = rd_w.getPlotItem()
            prd.setLabel('left', 'Resistance')
            prd.setLabel('bottom', 'Relative Displacement', units='mm')
            prd.setAxisItems({'left': ResistanceAxisItem(orientation='left')})
            prd.showGrid(x=True, y=True, alpha=0.3)
            # D) remove axis.setGrid(...) numeric calls
            prd.setClipToView(True)
            prd.setDownsampling(mode='peak', auto=True)
            prd.setYRange(1.0, 200.0, padding=0)
            prd.getViewBox().setLimits(xMin=0, yMin=0, yMax=500.0, minYRange=0.1, maxYRange=500.0)

    def _fmt_res_kohm(self, r_k):
        try:
            r = float(r_k)
        except Exception:
            return "--"
        if r >= 1000.0:
            return f"{r/1000.0:.3f} MΩ"
        elif r >= 1.0:
            return f"{r:.3f} kΩ"
        else:
            return f"{r*1000.0:.0f} Ω"

    def attach_hover_handlers(self):
        # Connect hover handlers with light throttling
        self.ui.adc_plot_widget.getPlotItem().scene().sigMouseMoved.connect(self._handle_adc_hover)
        self.ui.res_plot_widget.getPlotItem().scene().sigMouseMoved.connect(self._handle_res_hover)
        if hasattr(self.ui, 'v_vs_d_plot_widget') and self.ui.v_vs_d_plot_widget:
            self.ui.v_vs_d_plot_widget.getPlotItem().scene().sigMouseMoved.connect(self._handle_vd_hover)
        if hasattr(self.ui, 'r_vs_d_plot_widget') and self.ui.r_vs_d_plot_widget:
            self.ui.r_vs_d_plot_widget.getPlotItem().scene().sigMouseMoved.connect(self._handle_rd_hover)

    # ---------------- Public API ----------------
    def clear_all(self):
        self._t0 = time.time()
        self.adc_time.clear(); self.adc_volt.clear()
        self.res_time.clear(); self.res_val.clear()
        self._adc_t_arr = None; self._adc_v_arr = None
        self._res_t_arr = None; self._res_r_arr = None

        # Remove old curves from V–D / R–D
        for c in self.vd_curves:
            if hasattr(self.ui, 'v_vs_d_plot_widget') and self.ui.v_vs_d_plot_widget:
                self.ui.v_vs_d_plot_widget.removeItem(c)
        for c in self.rd_curves:
            if hasattr(self.ui, 'r_vs_d_plot_widget') and self.ui.r_vs_d_plot_widget:
                self.ui.r_vs_d_plot_widget.removeItem(c)
        self.vd_curves.clear(); self.rd_curves.clear()
        self.vd_legs.clear(); self.rd_legs.clear()
        self.vd_leg_colors.clear(); self.rd_leg_colors.clear()
        self.leg_max_dist_mm = 0.0

        # Clear time curves
        self.adc_curve.clear(); self.res_curve.clear()

    def append_live(self, voltage, resistance, t=None):
        if t is None: t = time.time()
        ts = t - self._t0
        if ts < 0:
            self._t0 = time.time()
            ts = 0.0
        self.adc_time.append(ts)
        self.adc_volt.append(voltage)
        self.res_time.append(ts)
        self.res_val.append(resistance)

    def set_leg_max_range(self, mm):
        try:
            self.leg_max_dist_mm = max(0.0, float(mm))
        except Exception:
            self.leg_max_dist_mm = 0.0

    # ============ B) clean start_new_leg (apply _config_curve, remove extra prune) ============
    def start_new_leg(self, cycle_index: int = 0):
        if cycle_index is None:
            color_idx = self.vd_leg_colors[-1] if self.vd_leg_colors else 0
        else:
            color_idx = int(cycle_index % len(self.cycle_pens))

        # V–D
        self.vd_legs.append([])
        self.vd_leg_colors.append(color_idx)
        pen_v = self.cycle_pens[color_idx]
        cv = self.ui.v_vs_d_plot_widget.plot(pen=pen_v)
        self._config_curve(cv)
        self.vd_curves.append(cv)
        # R–D
        self.rd_legs.append([])
        self.rd_leg_colors.append(color_idx)
        pen_r = self.cycle_pens[color_idx]
        cr = self.ui.r_vs_d_plot_widget.plot(pen=pen_r)
        self._config_curve(cr)
        self.rd_curves.append(cr)

        # فقط بر اساس 2*N لگ آخر prune کن
        self._prune_old_legs_if_needed()

    def set_keep_last_n_cycles(self, n: int):
        self.keep_last_n_cycles = max(1, int(n))
        self.max_legs = self.keep_last_n_cycles * 2
        if hasattr(self, '_prune_old_legs_if_needed'):
            self._prune_old_legs_if_needed()

    def append_displacement(self, distance_mm, voltage, resistance):
        if not self.vd_legs:
            self.start_new_leg(cycle_index=0)
        self.vd_legs[-1].append((distance_mm, voltage))
        self.rd_legs[-1].append((distance_mm, resistance))

    def update_plots(self):
        # --- Time plots (Voltage & Resistance vs Time) ---
        if self.adc_time:
            t = np.fromiter(self.adc_time, dtype=float)
            v = np.fromiter(self.adc_volt, dtype=float)
            r = np.fromiter(self.res_val,  dtype=float)
            self._adc_t_arr = t; self._adc_v_arr = v
            self._res_t_arr = t; self._res_r_arr = r

            self.adc_curve.setData(t, v, skipFiniteCheck=True)
            self.res_curve.setData(t, r, skipFiniteCheck=True)

            # smooth scroll
            max_t = float(t[-1])
            if self._adc_xmax_smooth == 0.0:
                self._adc_xmax_smooth = max_t
                self._res_xmax_smooth = max_t
            else:
                a = self.scroll_alpha
                self._adc_xmax_smooth = (1.0 - a) * self._adc_xmax_smooth + a * max_t
                self._res_xmax_smooth = (1.0 - a) * self._res_xmax_smooth + a * max_t

            tmax_a = self._adc_xmax_smooth
            tmax_r = self._res_xmax_smooth
            tmin_a = max(0.0, tmax_a - self.plot_time_window)
            tmin_r = max(0.0, tmax_r - self.plot_time_window)

            vb_a = self.ui.adc_plot_widget.getPlotItem().getViewBox()
            vb_r = self.ui.res_plot_widget.getPlotItem().getViewBox()
            vb_a.setXRange(tmin_a, tmax_a, padding=0.5)
            vb_r.setXRange(tmin_r, tmax_r, padding=0.5)

        # V–D curves
        if hasattr(self.ui, 'v_vs_d_plot_widget') and self.vd_legs is not None:
            # add missing curves (apply config)
            while len(self.vd_curves) < len(self.vd_legs):
                idx = len(self.vd_curves)
            for i, leg in enumerate(self.vd_legs):
                if leg:
                    x, y = zip(*leg)
                    self.vd_curves[i].setData(x, y, skipFiniteCheck=True)
                else:
                    self.vd_curves[i].clear()
            if self.leg_max_dist_mm > 0:
                self.ui.v_vs_d_plot_widget.setXRange(0, self.leg_max_dist_mm, padding=0.02)

        # R–D curves
        if hasattr(self.ui, 'r_vs_d_plot_widget') and self.rd_legs is not None:
            # add missing curves (apply config)
            while len(self.rd_curves) < len(self.rd_legs):
                idx = len(self.rd_curves)
            for i, leg in enumerate(self.rd_legs):
                if leg:
                    x, y = zip(*leg)
                    self.rd_curves[i].setData(x, y, skipFiniteCheck=True)
                else:
                    self.rd_curves[i].clear()
            if self.leg_max_dist_mm > 0:
                self.ui.r_vs_d_plot_widget.setXRange(0, self.leg_max_dist_mm, padding=0.02)

    def _prune_old_legs_if_needed(self):
        # V–D
        while len(self.vd_legs) > self.max_legs:
            self.vd_legs.pop(0)
            self.vd_leg_colors.pop(0)
            old_curve_v = self.vd_curves.pop(0)
            if hasattr(self.ui, 'v_vs_d_plot_widget') and self.ui.v_vs_d_plot_widget:
                self.ui.v_vs_d_plot_widget.removeItem(old_curve_v)

        # R–D
        while len(self.rd_legs) > self.max_legs:
            self.rd_legs.pop(0)
            self.rd_leg_colors.pop(0)
            old_curve_r = self.rd_curves.pop(0)
            if hasattr(self.ui, 'r_vs_d_plot_widget') and self.ui.r_vs_d_plot_widget:
                self.ui.r_vs_d_plot_widget.removeItem(old_curve_r)

    # ---------------- Hover handlers (optional) ----------------
    def _throttle_hover(self) -> bool:
        now = time.time()
        if now - self._last_hover_ts < self._hover_min_period:
            return False
        self._last_hover_ts = now
        return True

    def _handle_adc_hover(self, scene_pos):
        if not self._throttle_hover():
            return
        plt = self.ui.adc_plot_widget
        plot_item = plt.getPlotItem()
        vb = plot_item.getViewBox()
        if plt.sceneBoundingRect().contains(scene_pos) and self._adc_t_arr is not None and self._adc_t_arr.size > 0:
            p = vb.mapSceneToView(scene_pos)
            x = p.x()
            t_arr = self._adc_t_arr
            idx = int(np.searchsorted(t_arr, x))
            idx = max(0, min(len(t_arr) - 1, idx))
            if idx > 0 and abs(x - t_arr[idx - 1]) < abs(x - t_arr[idx]):
                idx -= 1
            nearest_t = t_arr[idx]
            nearest_v = float(self._adc_v_arr[idx])
            if nearest_t < 60:
                time_str = f"{nearest_t:.2f} s"
            elif nearest_t < 3600:
                m = int(nearest_t // 60); s = nearest_t % 60
                time_str = f"{m:d}:{s:05.2f} min"
            else:
                h = int(nearest_t // 3600); rem = nearest_t % 3600
                m = int(rem // 60); s = rem % 60
                time_str = f"{h:d}:{m:02d}:{s:02.0f} h"
            tooltip = f"Time: {time_str}\nVoltage: {nearest_v:.4f} V"
            QToolTip.showText(QCursor.pos(), tooltip, plt)
        else:
            QToolTip.hideText()

    def _handle_res_hover(self, scene_pos):
        if not self._throttle_hover():
            return
        plt = self.ui.res_plot_widget
        plot_item = plt.getPlotItem()
        vb = plot_item.getViewBox()
        if plt.sceneBoundingRect().contains(scene_pos) and self._res_t_arr is not None and self._res_t_arr.size > 0:
            p = vb.mapSceneToView(scene_pos)
            x = p.x()
            t_arr = self._res_t_arr
            idx = int(np.searchsorted(t_arr, x))
            idx = max(0, min(len(t_arr) - 1, idx))
            if idx > 0 and abs(x - t_arr[idx - 1]) < abs(x - t_arr[idx]):
                idx -= 1
            nearest_t = t_arr[idx]
            nearest_r = float(self._res_r_arr[idx])
            if nearest_t < 60:
                time_str = f"{nearest_t:.2f} s"
            elif nearest_t < 3600:
                m = int(nearest_t // 60); s = nearest_t % 60
                time_str = f"{m:d}:{s:05.2f} min"
            else:
                h = int(nearest_t // 3600); rem = nearest_t % 3600
                m = int(rem // 60); s = rem % 60
                time_str = f"{h:d}:{m:02d}:{s:02.0f} h"
            tooltip = f"Time: {time_str}\nResistance: {self._fmt_res_kohm(nearest_r)}"
            QToolTip.showText(QCursor.pos(), tooltip, plt)
        else:
            QToolTip.hideText()

    def _handle_vd_hover(self, scene_pos):
        if not self._throttle_hover():
            return
        if not hasattr(self.ui, 'v_vs_d_plot_widget') or not self.vd_legs:
            QToolTip.hideText(); return
        plt = self.ui.v_vs_d_plot_widget
        plot_item = plt.getPlotItem()
        vb = plot_item.getViewBox()
        if plt.sceneBoundingRect().contains(scene_pos):
            p = vb.mapSceneToView(scene_pos)
            x_dist = p.x()
            all_points = []
            for leg in self.vd_legs:
                all_points.extend(leg)
            if all_points:
                arr = np.array(all_points, dtype=float)
                idx = int(np.abs(arr[:, 0] - x_dist).argmin())
                nearest_dist, nearest_v = arr[idx]
                tooltip = f"Distance: {nearest_dist:.2f} mm\nVoltage: {nearest_v:.4f} V"
                QToolTip.showText(QCursor.pos(), tooltip, plt)
            else:
                QToolTip.hideText()
        else:
            QToolTip.hideText()

    def _handle_rd_hover(self, scene_pos):
        if not self._throttle_hover():
            return
        if not hasattr(self.ui, 'r_vs_d_plot_widget') or not self.rd_legs:
            QToolTip.hideText(); return
        plt = self.ui.r_vs_d_plot_widget
        plot_item = plt.getPlotItem()
        vb = plot_item.getViewBox()
        if plt.sceneBoundingRect().contains(scene_pos):
            p = vb.mapSceneToView(scene_pos)
            x_dist = p.x()
            all_points = []
            for leg in self.rd_legs:
                all_points.extend(leg)
            if all_points:
                arr = np.array(all_points, dtype=float)
                idx = int(np.abs(arr[:, 0] - x_dist).argmin())
                nearest_dist, nearest_r = arr[idx]
                tooltip = f"Distance: {nearest_dist:.2f} mm\nResistance: {self._fmt_res_kohm(nearest_r)}"
                QToolTip.showText(QCursor.pos(), tooltip, plt)
            else:
                QToolTip.hideText()
        else:
            QToolTip.hideText()