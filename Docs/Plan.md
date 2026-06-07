# プロンプト

Cinema4D2026.2.0を使っているけれど、「ポリゴンペン」に一部不具合があり使いづらい。
昔からある問題で、建物や町並みレベルの大きなモデルを扱いつつ、その中の細部、小さな家具や小物の内部の詳細な部分に対してポリゴンペンの「投影モード」を使っても、正しくメッシュ状にポリゴンが投影されずにカメラの手前の方の空中にポリゴンの点が指定されてしまう。
どんな視点においても正しくメッシュ状に投影しながらポリゴンが作成できる「ポリゴンペン改」ツールを作ってほしい。
機能は既存「ポリゴンペン」丸ごとそのまま同じでよい。「投影モード」のみ正しく動作する用になっていればよい。
なお「スナップ機能」は使わない。フォトグラメトリモデルを編集するのでスナップをオンにするととても負荷が高くなってしまうため。
ポリゴンペンの投影モードはスナップ機能ではないが実質面へのスナップとして機能しつつも動作が軽量なので重宝している。なのでこの特性を維持したままただしく投影できるようにしたい。
Cinema4Dのバージョンは2026.2.0。



# プラン Claude

# ポリゴンペン改 (PolygonPen Kai) — Cinema4D 2026.2.0 プラグイン実装プラン

## Context（背景・目的）

Cinema4D 2026.2.0 ネイティブ「ポリゴンペン」の**投影モードの不具合**：建物・町並み規模の巨大シーン内で、小さな家具・小物の表面に投影しようとすると、正しくメッシュ表面に投影されず**カメラ手前の空中に頂点が飛ぶ**。

**根本原因（見立て）**：投影モードはビューポートの深度バッファ(Zバッファ)依存と考えられる。深度バッファは 1/z の非線形分布のため、巨大シーン（far平面が極端に遠い）では遠〜中景の深度精度が破綻し、小物表面の深度を取り損ねる。Maxon自身も「ジオメトリは原点付近・数千単位以内に収めないと精度問題が出る」と注意している。

**なぜ自作するか**：ネイティブのポリゴンペンはC++製で、Python SDKから投影ロジックを継承・フック・部分差し替えする公開APIが存在しない（`ToolData` は新規ツールを一から作る枠組み）。よって**最低限の機能を自力実装**する（ユーザー合意済み）。

**解決策**：深度バッファを一切使わず、`c4d.utils.GeRayCollider` による**解析的レイ-ポリゴン交差**で投影する。レイと三角形の数学的交差なのでシーンスケール・near/far に完全非依存で常に正確。スナップ（吸着探索）ではなく単発レイ判定なので**軽量**＝ユーザーが重視する特性を維持できる。

**到達目標**：どんな視点・スケールでも下地メッシュ表面に正確に投影しながらポリゴンを作成できるツール。

## スコープ（確定仕様）

**含む**：
1. 投影クリックで頂点を打つ（GeRayColliderで下地表面の正確な交点へ）
2. 連続クリックで頂点を結びポリゴン面（三角/四角）作成、および**面の継ぎ足し**（既存頂点・エッジを起点に隣接面を連続生成）
3. **頂点のドラッグ投影移動**：打った頂点が投影ズレしたら掴んでドラッグ、ドラッグ中もメッシュ表面に投影し続け、マウスを離した位置の表面に確定

**含まない**：頂点削除・エッジ分割（既存ツールで行う）、スナップ機能

## 作成先の自動モード切替（確定）

クリック時の選択状態で自動判定：
- **何も選択なし or 編集不可オブジェクト選択** → **リトポ型**：下地（表示中の全ポリゴン系メッシュ）にレイ、**新規 PolygonObject** に作成
- **編集可能ポリゴン選択**（`op.IsInstanceOf(c4d.Opolygon)` かつ非ジェネレータ）→ **同一オブジェクト**：その選択メッシュにレイ＋直接描き足し

## ファイル構成（すべて新規）

```
Cinema4DPolygonPen/
├─ README.md                                  # 既存。インストール/使い方/ネイティブとの差分を追記
├─ SampleData/EirakuKan_Test_PolygonPen.c4d   # 既存。検証用
└─ polygonpen_kai/                            # ← Cinema4D の plugins/ に置くフォルダ
   ├─ polygonpen_kai.pyp                       # メイン：ToolData 本体＋全ロジック
   ├─ res/description/toolpolygonpenkai.res /.h  # オプションUI定義（DescriptionToolData）
   ├─ res/strings_ja-JP/ , res/strings_en-US/  # ローカライズ文字列
   └─ icons/polygonpen_kai.tif                 # 32x32 ツールアイコン
```

`.pyp` はC4D起動時に自動ロード。プラグインIDは暫定 `1000001`、本番は plugincafe.maxon.net で一意ID取得して `.pyp` と `.h` を差し替え。

## 実装の核心

### A. 投影レイキャスト（最重要）

スクリーン→ワールドレイは**2点法**（透視・平行投影の両対応）：
```python
p0 = bd.SW(c4d.Vector(mx, my, 0.0))
p1 = bd.SW(c4d.Vector(mx, my, 100000.0))
dir_w = (p1 - p0).GetNormalized()
```

GeRayColliderは **ray_p / ray_dir がオブジェクト座標系**なので、ワールドレイを逆行列変換してから渡す（裏取り済みの確定式）：
```python
inv = ~world_mg
ray_p_obj   = inv * p0_world                    # 点（並進込み変換）
ray_dir_obj = inv.MulV(dir_w).GetNormalized()   # 方向（並進除去 = inv ^ dir_w）
collider.Intersect(ray_p_obj, ray_dir_obj, 1.0e9, False)
hit = collider.GetNearestIntersection()         # dict: hitpos(obj座標), distance, f_normal, face_id ...
world_pos = world_mg * hit["hitpos"]            # ワールドへ戻す
```

複数下地は各々レイキャストし、**ワールド空間距離 `(world_pos - p0).GetLength()` 最小**を採用（オブジェクト空間 distance はスケール差で比較不可なので使わない＝スケール非依存の肝）。カーソル下に下地が無い場合は頂点を打たない（＝空中に点を作らないことが本ツールの主目的）。

### B. 下地解決（編集不可・パラメトリック対応）

`resolve_polygon_and_matrix(op, bd)` で任意オブジェクトから「GeRayColliderに渡せる PolygonObject ＋ ワールド行列」の**リスト**を得る。優先順：
1. 編集可能ポリゴン → そのまま、`op.GetMg()`
2. `op.GetDeformCache()`（デフォーマ適用後の実形状）
3. `op.GetCache(bd)`（プリミティブ/サブディビ/ジェネレータ）
4. `SendModelingCommand(MCOMMAND_CURRENTSTATETOOBJECT)`（**クローンに対して**、最終手段・重い）

**重要な落とし穴**：キャッシュ内の仮想オブジェクトは `GetMg()` が無効。ルート実オブジェクトの `GetMg()` を起点に、各キャッシュ階層の `GetMl()` を `accum = accum * GetMl()` と再帰合成してワールド行列を求める。

### C. 性能（ColliderCache）— 巨大メッシュ対策

`GeRayCollider.Init` を毎クリック走らせると数百万ポリゴンで致命的。**下地が変わった時のみ Init** し、オブジェクトごとにインスタンスをキャッシュ。ダーティトークン（`GetDirty(DIRTYFLAGS_DATA|MATRIX)` ＋ 頂点数）一致でスキップ、`force=False` の内部ダーティチェックと二段構え。リトポ型では下地（フォトグラメトリ）は描画中ダーティにならず再 Init されない＝設計の肝。複数下地はそれぞれ独立コライダを dict 保持。

### D. メッシュ構築

`ResizeObject(pcnt+1, vcnt)`（既存保持で末尾追加）→ `SetPoint`（world_pos を `~obj.GetMg()` でローカルへ）→（面化時）`c4d.CPolygon`（三角は d=c）→ `SetPolygon` → `obj.Message(c4d.MSG_UPDATE)` → `c4d.EventAdd()`。新頂点を打つ前に `bd.WS` で既存頂点をスクリーン投影し、PX_TOL ピクセル以内なら既存インデックスを再利用（継ぎ足し・閉じ・重複頂点回避）。

### E. 頂点ドラッグ投影移動

ヒットテスト（`bd.WS` でツール作成頂点をスクリーン投影 → PX_TOL 以内の最近を掴む）→ `win.MouseDragStart` → `while win.MouseDrag()==MOUSEDRAGRESULT_CONTINUE` で毎回 `raycast_scene` → `SetPoint` → `DrawViews(DRAWFLAGS_ONLY_ACTIVE_VIEW|DRAWFLAGS_NO_THREAD)` → `MouseDragEnd` で確定。ドラッグ全体を 1 Undo 単位、ESC で旧座標復元。表面ヒット無しのフレームは最後の有効位置を保持。

### F. クラス構造（`polygonpen_kai.pyp`）

- `ColliderCache`：GeRayColliderインスタンスのキャッシュ管理
- `BackdropResolver`：モード判定＋下地収集（`is_editable_polygon`, `collect_all_visible_polylike`, `resolve_polygon_and_matrix`）
- `MeshBuilder`：頂点/面の追記・頂点共有（`append_point`, `add_face`, `find_shared_vertex`）
- `PolygonPenKaiData(c4d.plugins.ToolData)`：本体。`InitTool / FreeTool / GetState / GetCursorInfo / MouseInput / Draw / Message / AllocSubDialog`。状態（pending頂点列・draw_op・mode・collider_cache）をインスタンス属性に保持（ツール選択中は生存）

### G. オプションUI（DescriptionToolData 推奨）

`.res/.h/.str` で宣言的に定義（C4D標準ツールと同じ見た目・属性マネージャ表示・ローカライズ容易）。最小項目：現在モード表示（読取専用）／面の頂点数 3/4 切替／投影対象（全シーン/選択のみ）／頂点掴みピクセル許容値 PX_TOL（既定 8px）。値は `data`(BaseContainer) 経由で各コールバックから参照。

## 状態遷移

```
IDLE/HOVER（GetCursorInfo で毎フレ raycast → プレビュー点・ラバーバンド描画）
 ├─ 左クリック & 既存(自作)頂点を PX_TOL 内でヒット → DRAGGING（MouseDrag ループで投影移動）→ IDLE
 ├─ 左クリック & raycast 成功 → 頂点追加、pending_idx へ
 │     └─ len == face_vcount(3/4) → 面化 ＋ carry_edge で継ぎ足し → IDLE
 ├─ 左クリック & 空クリック(hit=None) → 何もしない（空中に点を打たない）
 └─ 右クリック/ESC → pending 破棄 ／ Enter → 現状で面を閉じて確定
モード判定（クリック毎）：編集可能ポリゴン選択→samemesh ／ それ以外→retopo（初回クリックで InsertObject＋UNDOTYPE_NEWOBJ）
```

## エッジケース・要検証（実装時にCinema4D実機で確認）

- `bd.SW` の z 深度スケール（2点法で吸収するが z0/z1 具体値の安定性）
- `win.MouseDrag()` の戻り値タプル順 `(result, dx, dy, channel)`（バージョン差）
- CSTO フォールバックが一時 document への挿入を要するか
- 非一様スケール下の法線変換（表示用途なら `MulV` で実用十分、厳密には逆転置行列）
- リスク：巨大メッシュ Init 負荷（ColliderCache で緩和）／空の新規オブジェクト残留（FreeTool で削除）／Undo 整合（StartUndo/EndUndo で 1 操作 1 単位）

## 検証手順（ユーザー手動／Cinema4D MCP接続なし前提）

**インストール**：`polygonpen_kai/` フォルダをC4Dの `plugins` フォルダにコピー → C4D 2026.2.0 再起動 → Shift+C（コマンダ）で「PolygonPen Kai」が出れば登録成功。Pythonコンソールでロードエラーが無いことを確認。

`SampleData\EirakuKan_Test_PolygonPen.c4d` を開いて：
1. **回帰の核心**：建物全体が入る遠景にして小物表面をクリック → **カメラ手前の空中に点が飛ばない**こと（ネイティブ不具合が再現しない）
2. リトポ型：何も選択せずクリック → 新規オブジェクトに表面沿いの三角/四角面が生成
3. 同一：編集可能ポリゴン選択 → そのメッシュに直接追加、既存頂点共有で重複が増えない
4. ドラッグ：頂点を掴んで動かす → 常に表面に張り付き、離した位置の表面に確定。別面/別オブジェクト上へも乗り換え投影
5. 透視/平行投影の両方、極端な近接/遠方でも破綻しない
6. パラメトリック下地（プリミティブ/サブディビ）でもキャッシュ経由で投影
7. Undo（頂点/面/ドラッグ各1単位）、右クリック/ESC でキャンセル、離脱後に空オブジェクトが残らない
8. 性能：初回クリック（Init）後、2回目以降が即応（ColliderCache が機能）

## 着手順（推奨）

1. ToolData 骨格＋登録＋ `GetCursorInfo` ホバー投影（単一下地）で**投影が正しいことを最速確認**
2. `ColliderCache` ＋複数下地 `raycast_scene`
3. `MeshBuilder` ＋ retopo モードの頂点/面生成
4. samemesh モード判定＋継ぎ足し・頂点共有
5. 下地キャッシュ解決（プリミティブ/サブディビ対応）
6. 頂点ドラッグ
7. オプションUI（DescriptionToolData）
8. Undo 整合・性能チューニング