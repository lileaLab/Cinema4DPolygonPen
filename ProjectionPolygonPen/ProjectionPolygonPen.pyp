# -*- coding: utf-8 -*-
"""
ProjectionPolygonPen (投影ポリゴンペン)  for Cinema 4D 2026.2.0

ネイティブ「ポリゴンペン」の投影モードは、巨大シーン（建物・町並み規模）内の
小さなオブジェクト表面に使うと、深度バッファ精度の破綻でカメラ手前の空中に
頂点が飛ぶ不具合がある。本ツールは深度バッファを一切使わず、
c4d.utils.GeRayCollider による解析的レイ-ポリゴン交差で投影するため、
シーンスケール・視点に依存せず常に正確にメッシュ表面へ投影できる。
スナップ（吸着探索）ではなく単発レイ判定なので軽量。

操作:
  - 左クリック         : 下地メッシュ表面に投影して頂点を打つ（連続クリックで頂点列を作成）。
  - 最初の頂点をクリック : 3頂点以上たまっていれば多角形(N-gon)を閉じて作成・確定。
  - 既存頂点をドラッグ  : メッシュ表面に投影しながら移動、離した位置の表面に確定。
  - 既存頂点をクリック  : その頂点を共有（継ぎ足し）。
  - 右クリック / ESC    : 作成中の頂点列を破棄。

作成先の自動切替:
  - 何も選択なし / 編集不可オブジェクト選択 -> リトポ型: 下地にレイ、新規PolygonObjectに作成
  - 編集可能ポリゴン選択                    -> 同一オブジェクト: そのメッシュに直接描き足し

インストール:
  この ProjectionPolygonPen フォルダを Cinema 4D の plugins フォルダにコピーし、
  Cinema 4D を再起動。Shift+C のコマンダで "ProjectionPolygonPen" を検索して起動。

注意:
  PLUGIN_ID は暫定値。配布時は https://plugincafe.maxon.net で取得した一意IDに差し替えること。
"""

import math
import c4d
from c4d import plugins, utils, gui


# ----------------------------------------------------------------------------
# 定数
# ----------------------------------------------------------------------------
PLUGIN_ID = 1059357  # 暫定。配布時は plugincafe で取得した一意IDへ差し替え

# UI パラメータID
ID_MODE_LABEL   = 1000
ID_PROJ_SCOPE   = 1002
ID_PX_TOL       = 1003

# 投影対象スコープ（コンボ値）
SCOPE_SCENE     = 0
SCOPE_SELECTION = 1

# レイの最大長（オブジェクト座標系。十分大きく取りスケール非依存に）
RAY_LENGTH = 1.0e9
# ドラッグ判定のピクセル閾値（これ未満の移動はクリック扱い）
DRAG_THRESHOLD_PX = 3.0


# ----------------------------------------------------------------------------
# ジオメトリ・ヘルパー
# ----------------------------------------------------------------------------
def is_editable_polygon(op):
    u"""編集可能ポリゴンか（=同一オブジェクトモードにすべきか）を判定。"""
    if op is None:
        return False
    if not op.IsInstanceOf(c4d.Opolygon):
        return False
    # プリミティブやジェネレータ（編集不可）は除外
    if op.GetInfo() & c4d.OBJECT_GENERATOR:
        return False
    return True


def collect_visible_polylike(doc):
    u"""表示中の全オブジェクトを再帰収集（投影対象=全シーンの候補）。"""
    out = []

    def walk(op):
        while op is not None:
            if op.GetEditorMode() != c4d.MODE_OFF:
                out.append(op)
                walk(op.GetDown())
            op = op.GetNext()

    walk(doc.GetFirstObject())
    return out


def _collect_cache_polys(node, out):
    u"""キャッシュ階層（仮想オブジェクト）を再帰し、PolygonObjectを (poly, mg) で集める。

    キャッシュ内の仮想オブジェクトの GetMg() は多くのケースで有効なワールド行列を返す。
    （要検証: 複雑なジェネレータでズレる場合は GetMl() の親からの合成に切替える。）
    """
    while node is not None:
        deform = node.GetDeformCache()
        cache = node.GetCache()
        if deform is not None:
            _collect_cache_polys(deform, out)
        elif cache is not None:
            _collect_cache_polys(cache, out)
        else:
            if (node.IsInstanceOf(c4d.Opolygon)
                    and not node.GetBit(c4d.BIT_CONTROLOBJECT)
                    and node.GetPolygonCount() > 0):
                out.append((node, node.GetMg()))
        down = node.GetDown()
        if down is not None:
            _collect_cache_polys(down, out)
        node = node.GetNext()


def _csto_polygon(op):
    u"""最終手段: Current State to Object でポリゴン化（クローンに対して実行）。"""
    try:
        clone = op.GetClone(c4d.COPYFLAGS_NO_HIERARCHY)
        res = c4d.utils.SendModelingCommand(
            command=c4d.MCOMMAND_CURRENTSTATETOOBJECT,
            list=[clone],
            mode=c4d.MODELINGCOMMANDMODE_ALL,
            bc=c4d.BaseContainer(),
            doc=op.GetDocument())
        if res and isinstance(res, list) and len(res) > 0:
            r = res[0]
            if r is not None and r.IsInstanceOf(c4d.Opolygon):
                return r
    except Exception:
        pass
    return None


def resolve_polygons(op, bd):
    u"""任意オブジェクトから (PolygonObject, worldMatrix) のリストを得る。

    優先順: 編集可能ポリゴン -> DeformCache -> Cache -> CSTO(最終手段)
    """
    out = []
    if op is None:
        return out

    # 1. 編集可能ポリゴン
    if op.IsInstanceOf(c4d.Opolygon) and not (op.GetInfo() & c4d.OBJECT_GENERATOR):
        if op.GetPolygonCount() > 0:
            out.append((op, op.GetMg()))
        return out

    # スプラインは投影対象外
    if op.GetInfo() & c4d.OBJECT_ISSPLINE:
        return out

    # 2. デフォーマ適用後のキャッシュ
    deform = op.GetDeformCache()
    if deform is not None:
        _collect_cache_polys(deform, out)

    # 3. ジェネレータ/プリミティブのビルドキャッシュ
    if not out:
        cache = op.GetCache()
        if cache is not None:
            _collect_cache_polys(cache, out)

    # 4. 最終手段: CSTO
    if not out:
        poly = _csto_polygon(op)
        if poly is not None and poly.GetPolygonCount() > 0:
            out.append((poly, op.GetMg()))

    return out


def build_ray(bd, mx, my):
    u"""スクリーン座標 (mx,my) からワールド空間のレイ (始点, 正規化方向) を2点法で構築。

    透視・平行投影の両方で正しい方向が得られる。
    """
    p0 = bd.SW(c4d.Vector(float(mx), float(my), 0.0))
    p1 = bd.SW(c4d.Vector(float(mx), float(my), 100000.0))
    d = p1 - p0
    if d.GetLength() < 1.0e-9:
        return None, None
    return p0, d.GetNormalized()


def raycast_poly(collider, world_mg, p0_world, dir_world):
    u"""1つの下地ポリゴンにレイキャスト。最近交点を {world_pos, normal, face_id} で返す。"""
    inv = ~world_mg
    ray_p = inv * p0_world                  # 始点（点として変換, 並進込み）
    ray_d = inv.MulV(dir_world)             # 方向（並進除去）
    L = ray_d.GetLength()
    if L < 1.0e-12:
        return None
    ray_d = ray_d * (1.0 / L)

    if not collider.Intersect(ray_p, ray_d, RAY_LENGTH, False):
        return None
    hit = collider.GetNearestIntersection()
    if not hit:
        return None

    world_pos = world_mg * hit["hitpos"]
    n = hit.get("f_normal", None)
    if n is None:
        n = hit.get("s_normal", c4d.Vector(0.0, 1.0, 0.0))
    world_n = world_mg.MulV(n)
    if world_n.GetLength() > 1.0e-12:
        world_n = world_n.GetNormalized()
    return {"world_pos": world_pos, "normal": world_n, "face_id": hit.get("face_id", -1)}


# ----------------------------------------------------------------------------
# レイコライダ・キャッシュ（巨大メッシュ対策: 下地が変わった時のみ Init）
# ----------------------------------------------------------------------------
class ColliderCache(object):
    def __init__(self):
        self._map = {}  # key -> GeRayCollider

    def _key(self, poly):
        try:
            return poly.GetGUID()
        except Exception:
            return id(poly)

    def get(self, poly):
        key = self._key(poly)
        col = self._map.get(key)
        if col is None:
            col = c4d.utils.GeRayCollider()
            self._map[key] = col
            ok = col.Init(poly, True)    # 初回は強制ビルド
        else:
            ok = col.Init(poly, False)   # 以降はダーティチェックで高速にスキップ
        if not ok:
            return None
        return col

    def clear(self):
        self._map.clear()


# ----------------------------------------------------------------------------
# メッシュ構築
# ----------------------------------------------------------------------------
def append_point(obj, world_pos):
    u"""obj 末尾に1頂点を追加（world_pos をローカル座標へ変換）。新頂点indexを返す。"""
    local = (~obj.GetMg()) * world_pos
    n = obj.GetPointCount()
    obj.ResizeObject(n + 1, obj.GetPolygonCount())  # 既存保持で末尾追加
    obj.SetPoint(n, local)
    obj.Message(c4d.MSG_UPDATE)
    return n


def add_face(obj, idxs):
    u"""3 or 4 頂点から面を1つ追加。三角は d=c。"""
    a, b, c = idxs[0], idxs[1], idxs[2]
    d = idxs[3] if len(idxs) >= 4 else c
    pc = obj.GetPolygonCount()
    obj.ResizeObject(obj.GetPointCount(), pc + 1)
    obj.SetPolygon(pc, c4d.CPolygon(a, b, c, d))
    obj.Message(c4d.MSG_UPDATE)
    return pc


def add_ngon(obj, idxs, doc):
    u"""5頂点以上の多角形を作成。

    Cinema 4D のポリゴンは最大4頂点なので、まず頂点列を三角ファンで分割して
    複数の三角ポリゴンを作り、その後 UNTRIANGULATE で内部エッジを溶かして
    1枚の N-gon に統合する（非平面でも統合できるよう角度しきい値は最大に）。
    UNTRIANGULATE が効かない場合でも三角群として形状は残る。
    """
    n = len(idxs)
    pc = obj.GetPolygonCount()
    tri = n - 2
    obj.ResizeObject(obj.GetPointCount(), pc + tri)
    for k in range(tri):
        obj.SetPolygon(pc + k, c4d.CPolygon(idxs[0], idxs[k + 1], idxs[k + 2]))
    obj.Message(c4d.MSG_UPDATE)

    # 追加した三角を選択して N-gon 化（内部エッジを溶かす）
    try:
        sel = obj.GetPolygonS()
        sel.DeselectAll()
        for k in range(tri):
            sel.Select(pc + k)
        bc = c4d.BaseContainer()
        bc[c4d.MDATA_UNTRIANGULATE_ANGLE_RAD] = c4d.utils.DegToRad(179.999)
        c4d.utils.SendModelingCommand(
            command=c4d.MCOMMAND_UNTRIANGULATE,
            list=[obj],
            mode=c4d.MODELINGCOMMANDMODE_POLYGONSELECTION,
            bc=bc,
            doc=doc)
        sel.DeselectAll()
        obj.Message(c4d.MSG_UPDATE)
    except Exception:
        pass


def find_screen_vertex(obj, bd, mx, my, px_tol, start=0):
    u"""[start, 点数) の頂点のうちスクリーン上で px_tol 以内の最近頂点 index を返す（-1=なし）。

    巨大メッシュ（フォトグラメトリ）へ描き足す場合に備え、ツールで追加した頂点
    （start 以降）だけを走査して負荷を抑える。GetAllPoints() は全点コピーで重いので使わない。
    """
    mg = obj.GetMg()
    pc = obj.GetPointCount()
    best_i = -1
    best_d = float(px_tol)
    for i in range(start, pc):
        sp = bd.WS(mg * obj.GetPoint(i))
        dx = sp.x - mx
        dy = sp.y - my
        dd = math.sqrt(dx * dx + dy * dy)
        if dd < best_d:
            best_d = dd
            best_i = i
    return best_i


# ----------------------------------------------------------------------------
# オプションUI
# ----------------------------------------------------------------------------
class ProjectionPolygonPenDialog(gui.SubDialog):
    def __init__(self, td):
        self._td = td

    def CreateLayout(self):
        self.GroupBegin(0, c4d.BFH_SCALEFIT, 1, 0, "", 0)
        self.GroupBorderSpace(8, 8, 8, 8)

        self.AddStaticText(ID_MODE_LABEL, c4d.BFH_SCALEFIT, 0, 0, u"モード: -", 0)
        self.AddSeparatorH(0)

        self.GroupBegin(0, c4d.BFH_SCALEFIT, 2, 0, "", 0)
        self.GroupSpace(6, 4)

        self.AddStaticText(0, c4d.BFH_LEFT, 0, 0, u"投影対象", 0)
        self.AddComboBox(ID_PROJ_SCOPE, c4d.BFH_SCALEFIT)
        self.AddChild(ID_PROJ_SCOPE, SCOPE_SCENE, u"全シーン")
        self.AddChild(ID_PROJ_SCOPE, SCOPE_SELECTION, u"選択のみ")

        self.AddStaticText(0, c4d.BFH_LEFT, 0, 0, u"頂点掴み許容 (px)", 0)
        self.AddEditSlider(ID_PX_TOL, c4d.BFH_SCALEFIT)
        self.GroupEnd()

        self.GroupEnd()
        return True

    def InitValues(self):
        self.SetInt32(ID_PROJ_SCOPE, self._td.proj_scope)
        self.SetInt32(ID_PX_TOL, self._td.px_tol, 2, 40)
        self.update_mode_label()
        return True

    def update_mode_label(self):
        if self._td.mode == "samemesh":
            m = u"同一オブジェクト（描き足し）"
        else:
            m = u"リトポ（新規オブジェクト）"
        try:
            self.SetString(ID_MODE_LABEL, u"モード: " + m)
        except Exception:
            pass

    def Command(self, cid, msg):
        if cid == ID_PROJ_SCOPE:
            self._td.proj_scope = self.GetInt32(ID_PROJ_SCOPE)
        elif cid == ID_PX_TOL:
            self._td.px_tol = self.GetInt32(ID_PX_TOL)
        return True


# ----------------------------------------------------------------------------
# ツール本体
# ----------------------------------------------------------------------------
class ProjectionPolygonPenData(plugins.ToolData):

    # オプションと実行時状態はクラス属性でデフォルトを持つ。
    # ToolData の __init__ はオーバーライドしない（C4D のインスタンス化/登録を
    # 安全に保つため）。実体（cache 等）と毎セッションのリセットは InitTool で行う。
    proj_scope = SCOPE_SCENE
    px_tol = 8
    dialog = None
    cache = None
    mode = "retopo"
    draw_op = None       # 現在の作成先 PolygonObject
    _retopo_op = None    # リトポ用の新規 PolygonObject
    pending = None       # 確定待ち頂点 index 列（draw_op 基準）。InitTool で [] に
    hover_hit = None     # カーソル下の投影結果（プレビュー用）
    _base_pc = 0         # draw_op の「ツール開始時の頂点数」（掴み対象の下限）

    # --- 基本コールバック ---------------------------------------------------
    def GetState(self, doc):
        return c4d.CMD_ENABLED

    def InitTool(self, doc, data, bt=None):
        self.cache = ColliderCache()
        self.mode = "retopo"
        self.draw_op = None
        self._retopo_op = None
        self.pending = []
        self.hover_hit = None
        self._base_pc = 0
        return True

    def FreeTool(self, doc, data, bt=None):
        # 空のリトポオブジェクトを残さない
        if self._retopo_op is not None and self._alive(self._retopo_op):
            try:
                if self._retopo_op.GetPointCount() == 0:
                    self._retopo_op.Remove()
                    c4d.EventAdd()
            except Exception:
                pass
        self._retopo_op = None
        self.draw_op = None
        self.pending = []
        if self.cache is not None:
            self.cache.clear()

    def AllocSubDialog(self, bc):
        self.dialog = ProjectionPolygonPenDialog(self)
        return self.dialog

    # --- ヘルパー -----------------------------------------------------------
    def _alive(self, op):
        if op is None:
            return False
        try:
            return op.GetDocument() is not None
        except Exception:
            return False

    def _same(self, a, b):
        u"""同じ C4D オブジェクトか（Pythonラッパーが別インスタンスでも GUID で判定）。"""
        if a is None or b is None:
            return a is b
        try:
            return a.GetGUID() == b.GetGUID()
        except Exception:
            return a is b

    def _update_mode(self, doc):
        u"""選択状態からモードと作成先を決定（新規オブジェクトは作らない）。"""
        active = doc.GetActiveObject()
        if is_editable_polygon(active):
            self.mode = "samemesh"
            target = active
        else:
            self.mode = "retopo"
            target = self._retopo_op if self._alive(self._retopo_op) else None
        if not self._same(target, self.draw_op):
            self.draw_op = target
            self.pending = []
            self._base_pc = target.GetPointCount() if target is not None else 0
        elif target is not None:
            # 同じオブジェクトだがラッパーが変わった場合は参照だけ更新（pending/base_pc は維持）
            self.draw_op = target
        if self.dialog is not None:
            self.dialog.update_mode_label()

    def _ensure_target(self, doc):
        u"""クリック時: 作成先を保証（リトポで未作成なら新規生成）。"""
        self._update_mode(doc)
        if self.mode == "retopo" and self.draw_op is None:
            op = c4d.PolygonObject(0, 0)
            op.SetName("ProjectionPolygonPen")
            self._retopo_op = op
            self.draw_op = op
            self.pending = []
            self._base_pc = 0

    def _targets(self, doc):
        u"""投影対象（下地）オブジェクトのリスト。"""
        if self.mode == "samemesh":
            return [self.draw_op] if self.draw_op is not None else []
        # retopo: 作成先(_retopo_op)自身は下地から除外する
        if self.proj_scope == SCOPE_SELECTION:
            active = doc.GetActiveObject()
            if active is None or self._same(active, self._retopo_op):
                return []
            return [active]
        targets = collect_visible_polylike(doc)
        if self._retopo_op is not None:
            targets = [o for o in targets if not self._same(o, self._retopo_op)]
        return targets

    def _raycast(self, doc, bd, mx, my):
        u"""全下地にレイキャストし、ワールド距離が最も近い交点を返す。"""
        p0, dir_w = build_ray(bd, mx, my)
        if p0 is None:
            return None
        best = None
        best_d = None
        for op in self._targets(doc):
            for (poly, mg) in resolve_polygons(op, bd):
                col = self.cache.get(poly)
                if col is None:
                    continue
                hit = raycast_poly(col, mg, p0, dir_w)
                if hit is None:
                    continue
                d = (hit["world_pos"] - p0).GetLength()
                if best_d is None or d < best_d:
                    best_d = d
                    best = hit
        return best

    def _close_polygon(self, doc):
        u"""pending を閉じて多角形を作成し、作成終了（pending クリア）。"""
        n = len(self.pending)
        if n < 3:
            self.pending = []
            return
        doc.StartUndo()
        doc.AddUndo(c4d.UNDOTYPE_CHANGE, self.draw_op)
        if n <= 4:
            add_face(self.draw_op, self.pending)
        else:
            add_ngon(self.draw_op, self.pending, doc)
        doc.EndUndo()
        self.pending = []
        c4d.EventAdd()

    # --- マウス入力 ---------------------------------------------------------
    def MouseInput(self, doc, data, bd, win, msg):
        mx = int(msg[c4d.BFM_INPUT_X])
        my = int(msg[c4d.BFM_INPUT_Y])
        ch = msg[c4d.BFM_INPUT_CHANNEL]

        # 右クリック: 作成中の頂点列を破棄
        if ch == c4d.BFM_INPUT_MOUSERIGHT:
            self.pending = []
            c4d.EventAdd()
            return True

        if ch != c4d.BFM_INPUT_MOUSELEFT:
            return True

        self._ensure_target(doc)
        if self.draw_op is None:
            return True

        # 既存の自作頂点ヒット -> 閉じ / ドラッグ移動 / 共有
        if self._alive(self.draw_op):
            hit_idx = find_screen_vertex(self.draw_op, bd, mx, my, self.px_tol, self._base_pc)
            if hit_idx >= 0:
                # 作成中(3頂点以上)で最初の頂点をクリック -> 閉じる
                closing = (len(self.pending) >= 3 and hit_idx == self.pending[0])
                moved = self._drag_vertex(doc, bd, win, hit_idx, mx, my)
                if not moved:
                    if closing:
                        self._close_polygon(doc)
                    elif hit_idx not in self.pending:
                        # その他の頂点をクリック -> 共有（継ぎ足し）
                        self.pending.append(hit_idx)
                c4d.EventAdd()
                return True

        # 新規投影頂点
        hit = self._raycast(doc, bd, mx, my)
        if hit is None:
            # カーソル下に下地が無い -> 空中に点を打たない（本ツールの主目的）
            return True

        doc.StartUndo()
        if self.mode == "retopo" and not self._alive(self.draw_op):
            doc.InsertObject(self.draw_op)
            doc.AddUndo(c4d.UNDOTYPE_NEWOBJ, self.draw_op)
            # 注: retopo の新規オブジェクトはアクティブにしない。
            #     アクティブにすると次クリックで samemesh 判定になり下地を見失うため。
        else:
            doc.AddUndo(c4d.UNDOTYPE_CHANGE, self.draw_op)
        idx = append_point(self.draw_op, hit["world_pos"])
        self.pending.append(idx)
        doc.EndUndo()
        c4d.EventAdd()
        return True

    def _drag_vertex(self, doc, bd, win, idx, mx, my):
        u"""既存頂点を掴んでメッシュ表面に投影しながら移動。動いたら True。"""
        if not self._alive(self.draw_op):
            return False
        if idx >= self.draw_op.GetPointCount():
            return False

        old_local = self.draw_op.GetPoint(idx)
        win.MouseDragStart(c4d.KEY_MLEFT, float(mx), float(my),
                           c4d.MOUSEDRAGFLAGS_DONTHIDEMOUSE)
        cur_x = float(mx)
        cur_y = float(my)
        total = 0.0
        moved = False

        while True:
            result, dx, dy, channel = win.MouseDrag()
            if result != c4d.MOUSEDRAGRESULT_CONTINUE:
                break
            if dx == 0.0 and dy == 0.0:
                continue
            cur_x += dx
            cur_y += dy
            total += abs(dx) + abs(dy)
            if not moved and total < DRAG_THRESHOLD_PX:
                continue
            if not moved:
                moved = True
                doc.StartUndo()
                doc.AddUndo(c4d.UNDOTYPE_CHANGE, self.draw_op)
            hit = self._raycast(doc, bd, int(round(cur_x)), int(round(cur_y)))
            if hit is not None:
                self.draw_op.SetPoint(idx, (~self.draw_op.GetMg()) * hit["world_pos"])
                self.draw_op.Message(c4d.MSG_UPDATE)
                c4d.DrawViews(c4d.DRAWFLAGS_ONLY_ACTIVE_VIEW
                              | c4d.DRAWFLAGS_NO_THREAD
                              | c4d.DRAWFLAGS_NO_ANIMATION)

        end = win.MouseDragEnd()
        if moved:
            if end == c4d.MOUSEDRAGRESULT_ESCAPE:
                self.draw_op.SetPoint(idx, old_local)
                self.draw_op.Message(c4d.MSG_UPDATE)
            doc.EndUndo()
            c4d.EventAdd()
        return moved

    def KeyboardInput(self, doc, data, bd, win, msg):
        u"""ESC で作成中の頂点列を破棄。"""
        if msg.GetInt32(c4d.BFM_INPUT_CHANNEL) == c4d.KEY_ESC and self.pending:
            self.pending = []
            c4d.EventAdd()
            return True
        return False

    # --- カーソル情報 / プレビュー ------------------------------------------
    def GetCursorInfo(self, doc, data, bd, x, y, bc):
        self._update_mode(doc)
        self.hover_hit = self._raycast(doc, bd, int(x), int(y))

        if self.hover_hit is not None:
            bc[c4d.RESULT_CURSOR] = c4d.MOUSE_POINT_HAND
            bc[c4d.RESULT_BUBBLEHELP] = u"ProjectionPolygonPen: 投影OK"
        else:
            bc[c4d.RESULT_CURSOR] = c4d.MOUSE_FORBIDDEN
            bc[c4d.RESULT_BUBBLEHELP] = u"ProjectionPolygonPen: 下地メッシュなし"

        # 注: GetCursorInfo 内で DrawViews を呼ぶと描画が再帰的にネストしてクラッシュする。
        #     ホバープレビューの再描画は C4D の通常リフレッシュに任せ、ここでは呼ばない。
        return True

    # --- ビューポート描画 ---------------------------------------------------
    def Draw(self, doc, data, bd, bh, bt, flags):
        if not (flags & c4d.TOOLDRAWFLAGS_HIGHLIGHT):
            return c4d.TOOLDRAW_HANDLES

        bd.SetMatrix_Matrix(None, c4d.Matrix())

        # 確定待ち頂点列
        pts = []
        if self._alive(self.draw_op):
            mg = self.draw_op.GetMg()
            pc = self.draw_op.GetPointCount()
            for i in self.pending:
                if 0 <= i < pc:
                    pts.append(mg * self.draw_op.GetPoint(i))

        # 頂点列の線とハンドル
        bd.SetPen(c4d.Vector(1.0, 0.8, 0.0))
        for j in range(len(pts) - 1):
            bd.DrawLine(pts[j], pts[j + 1], 0)
        for p in pts:
            bd.DrawHandle(p, c4d.DRAWHANDLE_SMALL, 0)

        # 最初の頂点（閉じ先）を強調
        if len(pts) >= 1:
            bd.SetPen(c4d.Vector(1.0, 0.35, 0.0))
            bd.DrawHandle(pts[0], c4d.DRAWHANDLE_MIDDLE, 0)

        # カーソル下プレビュー点とラバーバンド
        if self.hover_hit is not None:
            hp = self.hover_hit["world_pos"]
            bd.SetPen(c4d.Vector(0.2, 1.0, 0.4))
            bd.DrawHandle(hp, c4d.DRAWHANDLE_MIDDLE, 0)
            if pts:
                bd.DrawLine(pts[-1], hp, 0)
                # 3頂点以上たまっていれば、閉じ線（最初の頂点へ）もプレビュー
                if len(pts) >= 2:
                    bd.SetPen(c4d.Vector(0.5, 0.5, 0.5))
                    bd.DrawLine(hp, pts[0], 0)

        return c4d.TOOLDRAW_HANDLES | c4d.TOOLDRAW_AXIS


# ----------------------------------------------------------------------------
# 登録
# ----------------------------------------------------------------------------
def main():
    # 注: c4d の登録関数は位置引数で呼ぶ（キーワード引数だと TypeError になる版がある）
    plugins.RegisterToolPlugin(PLUGIN_ID, "ProjectionPolygonPen", 0, None,
                               u"投影モードが正しく動作する投影ポリゴンペン（GeRayCollider投影）",
                               ProjectionPolygonPenData())


if __name__ == "__main__":
    main()
