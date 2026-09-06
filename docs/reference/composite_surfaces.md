# GEOUNED — Superficies compuestas: definición y construcción

> **Estado: v2 provisional — 2026-09-06.** Incorpora una primera ronda de
> correcciones del usuario (revisión sobre `.docx`).
> Sintetizado del registro de trabajo `CLAUDE.md`, reorganizado por tema
> (no cronológicamente). Redactado en un equipo cuyo checkout está en el commit
> `dcb774e` (rama `georeverse-migration`) y **sin acceso a las modificaciones sin
> comitear del otro equipo, ni al fichero `configuracion_cans.txt`, ni a la
> carpeta `memory/` donde vivía `composite_surface_definitions.md`**.
>
> Las afirmaciones marcadas **⚠ verificar** necesitan un repaso contra el código
> real (nombres de función/firma, estado actual de "en curso"/"revertido") en la
> fase 2, cuando haya acceso al otro equipo.
>
> Este documento pretende sustituir/consolidar al `composite_surface_definitions.md`
> referenciado en `CLAUDE.md`, que no está disponible aquí.

---

## 0. Marco general

### 0.1 Qué es una superficie compuesta

GEOUNED clasifica cada cara CAD en una de **5 superficies analíticas**: plano,
cilindro, cono, esfera, toro (`Gclassify_surface`, paquete `geo`).

Una **superficie compuesta / meta-superficie** *no* se modela en `geo`: la
ensambla el propio GEOUNED combinando varias analíticas mediante una expresión
booleana AND/OR. Tipos:

| Tipo | Idea de una frase | Corta el sólido en descomposición |
|---|---|---|
| **MultiPlane** | grupo de planos con normales casi coplanares, combinados AND u OR según convexidad | sí |
| **Can** (Fwd / Rev) | cilindro cerrado 360° + 2 superficies de cierre; la de cada lado tiene que ser **una única** superficie | sí |
| **TCone** (Fwd / Rev) | cono + 2 planos de cierre (config AND/OR por plano) | sí |
| **RoundCorner** | cilindro + 2 planos (normales ⊥ eje), cadena plano→cilindro→plano | sí |
| **MultiRoundCorner** | varios RoundCorner fusionados en una región; todas las normales de los planos bajo la misma convención, combinadas entre sí **exclusivamente en AND o en OR** (homogéneo) | sí |
| **ReversedConeCylinder** (RevCC) | el *exterior* de **uno o varios cilindros o conos** de ejes casi paralelos que se solapan | **no** — ver §6 |

### 0.2 Las clases `*Params` (`basic_functions_part1.py`)

`GeounedSurface.Surf` es siempre una de ~16 clases `*Params`, en 3 niveles
informales:

- **Tier-1 · descriptores puros**: `PlaneParams`, `CylinderOnlyParams`,
  `ConeOnlyParams`, `SphereOnlyParams`, `TorusOnlyParams`. Almacenan `GVector`
  (no `FreeCAD.Vector`). Storage-idénticos a `geo`'s `GPlane`/`GCylinder`/… pero
  aún son clases separadas.
- **Tier-2 · superficie de 2.º orden + plano(s) de cierre**: `CylinderParams`,
  `ConeParams`, `SphereParams`, `TorusParams`. Envuelven su primitiva un nivel
  más abajo como un `GeounedSurface` anidado (`.Cylinder.Surf.Cylinder`, etc.).
- **Tier-3 · compuestas** (combinan varias vía AND/OR): `MultiPlanesParams`,
  `CanParams`, `TConeParams`, `RoundCornerParams`, `MultiRoundCornerParams`,
  `ReversedConeCylParams`.
  - Naming de campos inconsistente para la misma idea:
    `s1`/`s1_configuration` vs `p1`/`p1_configuration` vs `Planes`+`Configuration`.

### 0.3 `bVar` / `region` / `components` y los dos ámbitos de numeración

Cada `GeounedSurface` lleva:

- **`.bVar`** — un `BoolVariable`, el id firmado con que se referencia la
  superficie.
- **`.region`** — un `BoolSurface` que envuelve un `BoolSequence`: la definición
  booleana AND/OR sobre los `bVar` de sus componentes (solo tipos compuestos).
- **`.components`** — `dict[abs(id), GeounedSurface]`: la relación
  numeración ↔ superficie, materializada en el objeto en vez de re-derivarse
  caminando `.Surf.X.Surf.Y` a mano.

**Se computan en dos fases con numeraciones genuinamente distintas. Mezclarlas
sería un error.**

| | **Fase de descomposición** | **Fase de conversión** |
|---|---|---|
| dónde | `utils/functions.py` construye el `GeounedSurface`; luego `build_surface()` → `build_complex_shape()` → `get_cell_object()` construye la forma CAD para cortar el sólido | `conversion/cell_definition.py` recorre cada cara de cada elemento descompuesto; `MetaSurfacesDict.add_*` (`geouned_classes.py`) |
| `.bVar` | id **local** desechable, secuencial, asignado al construir en `functions.py`. Solo mantiene la expresión AND/OR consistente para esa construcción CAD. Nada fuera de esa llamada depende de qué enteros son | se **sobreescribe in-place** con el id **global**, canónico y deduplicado: reusa un número existente si `is_same_plane`/`is_same_cylinder`/… (con tolerancia) encuentra un match geométrico en cualquier parte del modelo; incrementa `self.surfaceNumber` si no |
| `.region` | — | se computa aquí por primera vez |

**Consecuencia**: `get_cell_object` (descomposición) y
`MetaSurfacesDict.*_region` (conversión) **no pueden** hacer que uno lea la
`.region` ya computada del otro — cuando `get_cell_object` corre, la conversión
no ha ocurrido y `.region` no existe. Lo que **sí** comparten (y antes no) es la
**regla** AND/OR, que solo depende de qué ids / orientaciones / configuraciones
intervienen, nunca de si esos ids son locales o globales.

En la **fase de descomposición**, `.bVar` y `.region` sirven para definir la
expresión booleana de las superficies compuestas identificadas como caras del
sólido a descomponer. Esa expresión se usa para construir el objeto CAD que
corta el sólido original y lo descompone en sólidos irreducibles, mediante los
cortes que realiza la función `SplitSolids`.

En la **fase de conversión**, estas variables representan una superficie
**única**, definida para todo el modelo CAD que GEOUNED está convirtiendo. Todas
se representan mediante `.region` porque `region` es la variable elemental capaz
de almacenar tanto la definición de una superficie elemental como la de una
superficie compuesta.

### 0.4 La regla AND/OR como función pura

Las reglas AND/OR viven en funciones puras `(ids, config) -> BoolSurface` en
`basic_functions_part1.py`, llamadas **idénticamente** por `get_cell_object`
(descomposición) y por `MetaSurfacesDict.add_*` (conversión):

- `can_region(cid, cyl_orientation, surf_list)`
- `tcone_region(cid, cone_orientation, surf_list)`
- `round_corner_region(ids, configuration)`
- `multi_round_corner_region(...)`

`MetaSurfacesDict.Can_region`/`.TCone_region`/`.get_roundCorner_region` solo
resuelven ids (vía `primitive_surfaces.add_*`, incluido el "baile" de
volteo-de-signo-de-plano factorizado en `_resolve_plane_id`) y luego llaman a la
función pura.

### 0.5 Regla canónica del signo de la superficie característica

> Una superficie principal **Forward** aparece con id **negativo** en su propia
> región; **Reversed**, con id **positivo**.

De forma general, para todas las superficies **que no son planos**, la
orientación **Forward** indica que el material (el interior del sólido) está en
el lado *interior* de la superficie, y el id correspondiente es **negativo**;
**Reversed** corresponde a los casos en que el material está en la parte
*exterior* de la superficie, y el id es **positivo**.

Para los **planos**, **Forward** significa que el material está en el lado
*opuesto* a la dirección de la normal, y **Reversed** que el material está en el
*mismo* lado que la normal; también aquí Forward ⇒ signo negativo, Reversed ⇒
signo positivo. Esta convención de signo para planos **solo se cumple con la
orientación de los planos nativos**.

Ejemplo real (misma interfaz cilíndrica de radio 325 vista desde dos celdas
adyacentes):

```
Can_region(tubo hueco):    Orientation=Reversed  cid=4   region=OR[4,-3,-2]   -> signo de 4 = +1  (consistente)
Can_region(cilindro macizo): Orientation=Forward  cid=4   region=AND[2,3,-4]   -> signo de 4 = -1  (consistente)
```

`OR[4,-3,-2]` y `AND[2,3,-4]` son **complementos estructurales exactos**: la
misma interfaz física vista desde lados opuestos. Ambas tienen `.reverse=False`
(ninguna se construyó por negación) — esto **no** es una contradicción, aunque
`isSameInterface` históricamente lo trataba como tal.

`validate_characteristic_sign(region, surf_id, orientation, label)`
(`geouned_classes.py`, sobre un `literal_sign(region, surf_id)` en
`boolean_function.py`) valida esta regla en el momento en que la región se
finaliza; si una región viola su propia regla, es un bug real y lanza de
inmediato. `Can_region`/`add_forwardCan`/`add_reverseCan` pasan
`on_conflict="ignore"` a `isSameInterface` porque toda región que llega ahí ya
pasó esta validación por construcción.

### 0.6 Convención de normales, half-spaces y unidades

- La `.Axis` de un `GPlane` en GEOUNED **siempre apunta hacia el material**
  (convención distinta de la orientación de los planos nativos).
- `Gmake_half_space(plane)` se queda con el lado **−axis**.
- Para construir un half-space de corte: `GPlane.from_values(position,
  -material_normal)`, **nunca** `material_normal` directo.
- **Unidades**: GEOUNED trabaja internamente en **mm** (nativo STEP/FreeCAD); el
  input MCNP se escribe en **cm** (factor 0,1). Al comparar un punto real contra
  una superficie escrita en el `.mcnp`, escalar el punto por 0,1.

### 0.7 Cadena de código común (extremo a extremo)

1. **Detección** (fase descomposición): `decompose/generators.py::get_surfaces`
   → `next_Can` / `next_truncCone` / `next_roundCorner` /
   `exclude_no_cutting_planes` / `next_multiplanes` / … →
   `get_can_surfaces` / `get_tcone_surfaces` / `get_roundcorner_surfaces` /
   `multiplane()` (`meta_surfaces.py`) + helpers en `meta_surfaces_utils.py`.
   **RevCC es la excepción** (§6): no se detecta aquí.
2. **Construcción de params**: `build_can_params` / `build_roundC_params` /
   `build_tcone_params` / `build_RCC_params` (`utils/functions.py`).
3. **Regla AND/OR**: funciones puras `*_region` (§0.4).
4. **Forma CAD** (descomposición): `GeounedSurface.build_surface` →
   `build_complex_shape` (`build_shape_functions.py`) → `get_cell_object` /
   `BuildDepth` / `SplitSolid` (`build_region/`); constructores `makeCan` /
   `makeTCone` / `makeRoundCorner` / `makeMultiRoundCorner` / `makeMultiPlanes`.
5. **Registro global** (conversión): `MetaSurfacesDict.add_can` / `add_tcone` /
   `add_roundcorner` / `add_multiroundcorner` / `add_reversedCC`
   (`geouned_classes.py`).
6. **Clasificación de punto** — fallback algebraico
   (`boolean_solids.py::check_sign`), genérico y guiado por `.components`:
   ```python
   surfSet = {id: check_sign(point, comp) > 0 for id, comp in surf.components.items()}
   surf.region.region.evaluate(surfSet)
   ```

---

## 1. MultiPlane

### Definición

Un grupo de planos cuyas normales están **aproximadamente en un plano común**
(porque las superficies que acotan tienen ejes casi paralelos). Se clasifica por
un **test global de grupo**, no incremental:

1. Confirmar primero que la intersección/unión forma un arreglo **convexo**.
2. Clasificar: normales apuntando **hacia afuera** del centroide del grupo ⇒
   **OR** (arreglo abierto); **hacia adentro** ⇒ **AND**.

Implementado en `convex_planes(planes, axis) -> (convex, orientation)`
(`functions.py`): `orientation=="Forward"` ⇒ OR, `"Reversed"` ⇒ AND. La misma
función se reutiliza para RoundCorner/MultiRoundCorner y RevCC.

### Detección

`next_multiplanes` → `multiplane(master_plane, …)` (`meta_surfaces.py`): recorre
las aristas de cada plano; para cada arista `GLine` busca un plano adyacente y
encadena. `exclude_no_cutting_planes` → `external_plane()` (lógica basada en
`region_sign`) marca planos "externos" antes de que `next_multiplanes` corra.
Camino de conversión propio: `utils/functions.py::get_multiplanes` (desde
`conversion/cell_definition.py`).

### Trampas conocidas (**⚠ verificar estado actual**)

- `multiplane()` comparaba una **instancia contra una clase**
  (`Gclassify_curve(e) is GLine` en vez de `type(...) is GLine`) — hacía
  MultiPlane **indetectable para cualquier entrada**. Corregido a
  `if type(type_curve) is not GLine`. (`multiplane_old` / `multiplane_loop` son
  código muerto con el mismo bug, sin llamantes.)
- `convex_planes` tenía un test de signo **muerto** (`cross.dot(ref)` siempre
  `0`; debía ser `cross.dot(zaxis)`) y **no normalizaba** `atan2` a `[0, 2π)`
  antes de ordenar. Ambos corregidos. `build_roundC_params` además alimentaba a
  `convex_planes` con pocos puntos (solo los planos de esquina deduplicados);
  ahora también le pasa los planos de cierre `gpa` de cada cilindro para el test
  de convexidad/orientación (no para los términos AND/OR de nivel superior).
- El heurístico rotacional original de `convex_planes` es incorrecto en general
  (devuelve "convexo" para conjuntos de planos demostrablemente no acotados). Se
  prototipó un reemplazo general (`convex_polyhedron_from_halfspaces`,
  Sutherland–Hodgman) que se revirtió; el heurístico sigue en uso.

---

## 2. Can (Forward / Reversed)

### Definición

Un **cilindro cerrado 360°** cortado por **2 superficies de cierre** (`s1`,
`s2`). Cada cierre puede ser plano, cilindro, cono o esfera — no solo plano;
hace falta que el extremo entero sea **una única** superficie coherente.

- Para que haya un Can tiene que haber un cilindro **cerrado de verdad**. Un
  cilindro de 180°, o una cara con contorno jaggy (restos de un corte previo), no
  cuentan. El cierre topológico se verifica por **winding**
  (`_is_closed_by_winding`, `meta_surfaces_utils.py`): recorre el wire externo y
  exige que cada tramo de mismo sentido cierre a un múltiplo limpio de `2π`. No
  basta con que el bounding box UV abarque `2π`.
- Las piezas de un cilindro **partido** por un corte previo se re-fusionan
  (`merge_same_surface_faces` / `closed_cylinder_cone` → `ShellGu`) *antes* de
  comprobar el cierre y de buscar los cierres.

### Reglas AND/OR — la rejilla 2×2

Fuente autoritativa: `configuracion_cans.txt` (usuario; **no accesible aquí**,
ver §8). Resumen de lo que `CLAUDE.md` reproduce:

Ejes: **orientación del cilindro principal** × **configuración AND/OR de la
superficie secundaria `si`**.

| | `si` = AND | `si` = OR |
|---|---|---|
| cilindro **Forward** | ✅ emparejamiento normal | ⚠ **caso especial real** (cambiado respecto a `configuracion_cans.txt`, en una versión posterior): la extensión natural de `si` sustituye a un cierre ausente, con la **orientación opuesta** |
| cilindro **Reversed** | ⚠ **caso especial real** ("boca abierta"): la extensión natural de `si` sustituye a un cierre ausente; fórmula construida a partir de la fórmula de normal de la orientación **opuesta** | ✅ emparejamiento normal |

> **Nota (corrección del usuario)**: el `configuracion_cans.txt` original marcaba
> `Forward`+`OR` como "no es un Can" (rechazar); una versión posterior lo
> convirtió en un caso especial real, análogo al de `Reversed`+`AND`. Ahora
> **ambas** casillas fuera de la diagonal son casos "boca abierta".

**Regla canónica cono + su propio plano de ápice**: *siempre* `AND[-s, ap]` o
`OR[s, -ap]` — nunca las otras dos combinaciones de signo.
`cone_apex_plane()` debe devolver **incondicionalmente** `cone.Surface.Axis`
como normal del plano de ápice; todo el manejo de signo dependiente de
orientación vive en las fórmulas que lo consumen (`can_region`, `add_cone`).

### `omit` — normalización de orientación (**⚠ EN CURSO, con regresiones**)

El flag `omit=False` se activa en los dos casos fuera de la diagonal de la
rejilla (cilindro `Forward` con `si=OR`, y cilindro `Reversed` con `si=AND`).
Significa que esas superficies `si` **no pueden excluirse** del conjunto de
superficies de corte candidatas para las iteraciones siguientes, porque no
provienen de una cara real del sólido sino de la **extensión** de una parte
existente.

`region_sign`'s resultado AND/OR se normaliza contra la orientación del cilindro
principal (`Forward` ↔ `AND`, `Reversed` ↔ `OR`), volteando `r` y registrando el
volteo en un flag **`omit`** (3.er elemento de tupla `(s, r, omit)`, antes
`(s, r)`). Cuando `omit == False`, `build_can_params` voltea también la
normal/`Orientation` de la secundaria para compensar (la región física no
cambia, solo la representación se vuelve uniforme).

> **⚠** Rework no verificado del todo. Se sabe que la rama `omit=False` de
> `build_can_params` produce una fórmula "con fuga" en al menos un caso real
> ("Barrel upper left" de `TVA_final_allencl.stp`: el flip convierte un
> `Forward` real correcto en `Reversed` mal reportado, y `can_region` lo toma
> al pie de la letra). Regresiones históricas asociadas: `rev_can_1.stp`,
> `tank.stp` (esta última resultó ser un problema torus separado, resuelto).
> **Objetivo pendiente**: re-derivar la rama `omit=False` contra CAD real, con
> `TVA_final_allencl.stp` solid 9 como reproducción pequeña. Verificar en fase 2
> si esto ya se cerró.

### Detección

`next_Can` / `get_Can` (`generators.py` / `functions.py`) →
`get_can_surfaces(seed_face, solidFaces)` (`meta_surfaces.py`). Diseño: se le
pasa **cualquier** cara del cilindro y fusiona internamente; los llamantes no
pre-fusionan.

- Rama "cilindro adyacente mismo radio" (cilindro **quebrado** / kink — 3 tramos
  rectos del mismo radio con ejes que se cruzan en ángulo): exige **solo radio
  igual**, no paralelismo. La condición `is_parallel` que había ahí era código
  muerto para ese escenario (toda cara que llega a esa rama ya pasó la exclusión
  `is_same_surface`, así que nunca podía ser paralela y no-colineal). Devuelve
  `(s, None, True)`.
- `commonEdge` se comprueba contra el **`cylinder_shell` fusionado**, no contra
  la cara semilla — el borde compartido puede pertenecer a otra pieza del mismo
  cilindro fusionado.
- `get_can_surfaces` debe devolver exactamente 3 elementos (`cylinder_shell` + 2
  cierres); si no, rechaza limpiamente (un Can sin exactamente 2 cierres nunca
  fue un Can).
- Rama de washer-plane: `commonEdge` con `outer2_only=False` incondicional en
  `get_can_surfaces` (un plano de cierre anular puede compartir su borde con el
  cilindro por su wire *interior*, no el exterior).

### Construcción CAD / registro

`makeCan` → `build_complex_shape` → `get_cell_object` rama `"Can"`. Registro:
`MetaSurfacesDict.Can_region` / `add_forwardCan` / `add_reverseCan` →
resuelve ids (`_resolve_plane_id`) → `can_region(cid, cyl_orientation,
surf_list)`.

### Trampas conocidas (**⚠ verificar**)

- `can_region`, rama **cono + plano-ápice sin plano de cierre**, caso
  **Reversed**: usaba `+apid` (OR) donde el signo físico exige `-apid`. Fix
  aplicado. Segunda instancia de la misma clase en la sub-rama OR-configurada de
  la rama "ambos presentes / Reversed".
- `MetaSurfacesDict.add_cone` (Tier-2 standalone, independiente de `can_region`),
  rama Reversed: `cone_region + (-pid)` (con negación), no `+ pid`.
- El histórico `check_sign` casaba `si.Type == "cylinder"` (minúscula) contra el
  tag real `"Cylinder"` — un componente Can de tipo cilindro se caía
  silenciosamente del `surfSet`. Cerrado al pasar `.components` a un dict
  genérico.

---

## 3. TCone (Forward / Reversed)

### Definición

Un **cono** cortado por **2 planos de cierre**, cada uno con su **propia**
configuración AND/OR: `p1_configuration`, `p2_configuration`. Dato real por
plano, **no** derivable de la orientación del cono.

### Regla / registro

`tcone_region(cid, cone_orientation, surf_list)` (`basic_functions_part1.py`,
junto a `can_region` / `round_corner_region`). Detección: `next_truncCone` →
`get_tcone_surfaces`; params: `build_tcone_params` (`functions.py`).

### Trampas conocidas (**⚠ verificar**)

- `MetaSurfacesDict.TCone_region` calculaba un **único** `configuration` para
  *ambos* planos a partir solo de la orientación del cono
  (`"AND" if Forward else "OR"`), **ignorando** `p1_configuration` /
  `p2_configuration`. `get_cell_object` rama `"TCone"` *sí* usaba los datos
  reales → `check_sign` (que lee `.region`) y el sólido CAD de referencia
  evaluaban expresiones distintas para la misma superficie física. Fix:
  `TCone_region` hace ahora `zip((p1, p1_configuration), (p2, p2_configuration))`,
  igual que `Can_region`.
- Hubo un desajuste de aridad de tupla entre `get_tcone_surfaces` (3-tupla nueva)
  y `build_tcone_params` (esperaba 2-tupla) que se corrigió y **se revirtió** a
  petición del usuario para seguir con otra investigación — `build_tcone_params`
  puede estar roto así si se ejercita esa ruta. **Verificar en fase 2.**
- `TCone_region` no se ha extendido con `validate_characteristic_sign` /
  `on_conflict="ignore"` como `Can_region` — candidato natural si aparece un
  falso positivo de `isSameInterface`.

---

## 4. RoundCorner

### Definición

> **Estrictamente**: un **cilindro** + **dos planos** (normales *perpendiculares
> al eje del cilindro*), encadenados **plano → cilindro → plano**. Un "corner
> acotado por cono" **no tiene sentido** para este tipo.

Caso especial `p1 ≡ p2` (mismo plano de cierre en ambos extremos):

- **válido**: 2 caras *disjuntas* del mismo plano coincidente.
- **inválido**: genuinamente **1 sola cara real** cerrando ambos extremos. El
  "plano adicional" `pd` (a través de ambas aristas de contacto) *es* esa misma
  cara ⇒ probar p1/p2 contra `pd` es probar la cara contra sí misma;
  `AND_p1_pd` / `AND_p2_pd` salen como **opuestos lógicos incondicionales**
  (singularidad angular de 0°, no ruido de tangencia).
  Se rechaza en `get_adjacent_cylplane` (rama `cornerPlanes=True`): deduplica los
  planos de esquina por `Index` de cara real; si ambas aristas cierran contra la
  misma cara, colapsa a 1 entrada y el `len(adjacent_planes) != 2` del llamante
  lo rechaza solo.

### Configuración (`cyl_plane_region_conf`, `meta_surfaces_utils.py`)

Devuelve los flags `AND_p1_cyl` / `AND_p2_cyl` / `AND_p1_pd` / `AND_p2_pd` que
seleccionan una rama de `round_corner_region`.
`Configuration = 7` ⇒ `fwd_cyl + AND_p1_cyl + AND_p2_cyl` (todo-AND): la
clasificación natural de un poste acotado por 2 paredes paralelas.

La normal apuntando-a-material de cada plano de esquina (`n1` / `n2`) se deriva
históricamente por una heurística local
(`pr1 = ac1 × p1_axis`, `pr2 = -ac1 × p2_axis` — nótese el `-` **asimétrico**
en p2). La convención canónica pre-migración, ya implementada en
`conversion/cell_definition_functions.py::gen_plane`, es:
`material = -face.Surface.Axis if Orientation == "Forward" else face.Surface.Axis`.

### Trampas conocidas (**⚠ verificar**)

- **Cilindro partido**: `cyl_plane_region_conf` usaba la **cara semilla** para
  los vectores de referencia de *ambos* extremos. Cuando el cilindro del round
  corner está partido en 2 piezas de 90° (`merge_same_surface_faces`), cada
  extremo tiene su propia pieza adyacente real (`ep1[0]` / `ep2[0]`, antes
  descartadas). El punto de tangencia real de p2 puede caer en la *otra* pieza
  (u≈270°), no en la semilla (u≈90°). **Fix**: desempaquetar `cyl1` / `cyl2` de
  `ep1[0]` / `ep2[0]`; tomar `r1/nc1/nt1` del `ParameterRange` de `cyl1` y
  `r2/nc2` del extremo *lejano* de `cyl2`. Las fórmulas de `pr1`/`pr2` no se
  tocan — es un fix de "evaluar en el punto correcto".
- **Banda near-tangente**: cuando `cross1` / `cross2` cae en `[1e-8, 1e-3]` (por
  encima del guard degenerado de `1e-8`, pero numéricamente sin sentido),
  fallback por **muestreo de material real** (`_and_or_by_material_sampling`,
  ~600 puntos): decide AND vs OR comparando pertenencia real al sólido
  condicionada al lado del *otro* plano de esquina (cuyo signo no depende del
  cross product frágil). Requiere el `SolidGu`/`GSolid` que encierra la
  geometría — hilado por `get_roundCorner` / `next_roundCorner` /
  `get_roundcorner_surfaces` / `cyl_plane_region_conf`.
- Rama `p1id == p2id` de `round_corner_region`: aplicaba `fwd_cyl` **dos veces**
  (una en la pre-negación de `p1id`/`p2id`, otra en el `-rc_region` final). La
  sub-rama `AND` quedaba rota (78,7 % vs 99,9 % contra CAD real); la sub-rama
  `OR` no. **Fix**: capturar `p1id_raw` *antes* de la pre-negación y usarlo solo
  dentro de `if p1id == p2id: if AND_p1_cyl:`. Las demás ramas intactas.
- `get_overlap_rc` (~145 líneas, fusionar 2 round corners adyacentes) era
  **código muerto** (cero llamantes) — borrado.

### Bug fundacional relacionado — `is_same_plane_surface` (`geo/surface_geometry.py`)

Comparaba el offset de dos planos con ejes **antiparalelos** como `d1 == d2` en
vez de `d1 == -d2`. Rompía la detección "p1 ≡ p2": las 2 paredes paralelas
reales de `rc9.stp` se tomaban como el mismo plano. **Fix**: ramificar según el
signo del producto escalar de ejes (`d1 == d2` si paralelos, `d1 == -d2` si
antiparalelos). Función única compartida por los 3 motores.

### "4 configuraciones" — criterio de degeneración del hinge

Derivación dictada por el usuario (confirmada correcta), aunque el fix de código
que salió de ella se **revirtió**:

> **Config 4 (degenerada)**: la dirección de rayo elegida del semi-plano viaja
> hacia el *otro* punto de cruce del plano con el círculo del round corner, **y**
> ese segundo cruce cae *dentro* del arco del round corner. Ambas condiciones a
> la vez.

El fix `_plane_recrosses_arc` (rechazar cuando p1 y p2 ambos caen en este
patrón) se revirtió: rechazar un candidato en tiempo de descomposición desvía
silenciosamente `generic_split` por otra ruta de corte (mismo riesgo que el
precedente `get_can_surfaces`/`outer2_only`). El fix real fue
`is_same_plane_surface` (arriba) + `cyl_plane_region_conf`.

### Receta: cilindro oculto + planos tangentes (superficie no presente como cara real)

Cuando la superficie del round corner solo está *implícita* — 2 aristas de arco
parciales, mismo eje, centros alineados en el eje, mismo radio, sin ninguna cara
real en esa superficie:

1. En cada endpoint de cada arista de arco, el plano tangente ahí (normal
   material = dirección radial, signo apuntando hacia el *otro* punto tangente)
   ⇒ `p1` / `p2`.
2. El plano por los **4 vértices relevantes** (2 endpoints × 2 extremos axiales),
   que contiene el eje y la cuerda de puntos tangentes, es el plano adicional
   `pc`. Su signo, de datos reales / usuario — **no** de una heurística
   "empuja-un-punto-de-borde-hacia-dentro" (dio el signo equivocado la primera
   vez).
3. `region = box AND (cylinder OR pc) AND p1 AND p2`, con `box` = el BoundBox
   exacto del sólido de trabajo, **sin padding**.

---

## 5. MultiRoundCorner (MRC)

### Definición

Varios RoundCorner fusionados en una sola región. Comparte la regla AND/OR de la
forma correcta (`round_corner_region` / `multi_round_corner_region`, funciones
puras de ids), llamada igual por `get_cell_object` y por
`MetaSurfacesDict.add_multiroundcorner`.

**Condición de definibilidad**: con todas las normales de los planos orientadas
según la misma convención, todas las esquinas del grupo confirman
*independientemente* la **misma** relación —**todo AND o todo OR**, de forma
homogénea— con las paredes. Es la precondición para fusionarlas en un MRC en vez
de dejarlas como RoundCorner separados.

### `.components` / registro

Construido en `MetaSurfacesDict.get_roundCorner_region` (extendido a MRC). La
rama de `check_sign` está **fusionada** con la de RoundCorner:
`elif surf.Type in ("RoundCorner", "MultiRoundCorner"):`, ambas guiadas por
`.components` de forma idéntica.

### Trampas conocidas (**⚠ verificar**)

- Al añadir `.components` a MRC salieron **3 bugs pre-existentes** de path de
  atributo — `add_multiRoundCorner` nunca se había ejercitado end-to-end (su
  propio comentario lo admitía: *"will not return correct results for any multi
  corner configuration"*):
  - `cylplane.Axis = -cylplane.Axis` (debía `cylplane.Surf.Axis`)
  - `rc.Surf.Plane.bVar = pcid` (`RoundCornerParams` no tiene `.Plane`, solo
    `.Planes` / `.Cylinder` — debía `cylplane.bVar`)
  - la rama MRC de `check_sign` leía `rc.Surf.Plane` / `rc.Surf.Cylinder`
    directos — corregido a `rc.Surf.Cylinder.Surf.Plane` /
    `rc.Surf.Cylinder.Surf.Cylinder`.
- **⚠ Caveat importante**: la corrección de la rama MRC de `check_sign` **no**
  está verificada end-to-end contra geometría real — ningún fixture disponible
  fuerza esa ruta (`Gsplit` resuelve el corte antes de necesitar el fallback
  algebraico de `check_sign`). Es correcta *relativa al patrón establecido* de
  `get_cell_object` y ya no crashea; los *valores* que produce no están
  confirmados.
- `AdjacentMultiplanePlanes` (§6) está cableado solo para RevCC — **necesita
  extenderse a MRC**. Pendiente rastreado.

---

## 6. ReversedConeCylinder (RevCC)

### Definición

> El **exterior** de varios cilindros cuyos ejes son **casi (no necesariamente
> exactamente) paralelos** y que se **solapan** entre sí.

**Único entre los tipos compuestos**: RevCC **nunca corta** el sólido. No se
identifica en `decompose/generators.py::get_surfaces`. Solo se identifica en el
**módulo de conversión** (`add_reversedCC`, `geouned_classes.py`), recorriendo
las caras de un elemento *ya descompuesto* para reconstruir su expresión CSG —
describe una superficie de acotación externa de la celda *a posteriori*.
Escanear ficheros STEP vía `decompose_solids()` / `build_solid_definition()` y
buscar `RevCC` en `geo.Surfaces` **siempre da cero**, sea cual sea el corpus.

### Detección

`get_reversed_cone_cylinder` / `get_revConeCyl_surfaces` → `get_join_cone_cyl`
(`meta_surfaces_utils.py`): sigue una cadena cilindro + cono + cilindro con ejes
casi paralelos.

- `_valid_chain_junction` valida cada unión por un **criterio topológico**: en
  cada una de las dos caras candidatas, las 2 aristas (aparte de la compartida)
  que tocan los 2 endpoints de la arista compartida deben ser **aristas
  distintas** — no la misma arista (cara-"bigon" degenerada).
- Piso de alineación de ejes: `abs(axis1 · axis2) > 0.1` (permisivo; solo
  rechaza los ~6° antes de la perpendicularidad exacta).
- La curva de tangencia entre dos cilindros/conos *skew* (ejes no coplanares) es
  intrínsecamente un `GBSpline`, nunca recta — no filtrar por "arista recta".

### Regla de región (`add_reversedCC`, `geouned_classes.py`)

Rediseño (fix del usuario). **Antes**: cada segmento emparejado con su plano de
cierre *individual*
(`terms_region = AND[(s_i OR -p_i)]`, `plane_region = OR[p_1..p_n]`,
`region = plane_region * terms_region`).
**Ahora**: cada segmento emparejado con la **unión global** de todos los planos
de cierre. Algebraicamente, como la expresión final ya está AND-eada con
`plane_region`, se simplifica a:

> **`plane_region AND (s_1 AND s_2 AND … AND s_n)`**
> — material = estar en el lado correcto de *al menos un* plano de cierre (OR,
> sin cambio), **y** satisfacer *todas* las superficies primitivas de los
> segmentos a la vez (AND).

```python
plane_region = None
surf_components = []
for cc in cylcones:
    s_region, p_region = self._reversedCC_component(cc)
    plane_region = BoolSurface.add(plane_region, p_region)
    surf_components.append(s_region)

surf_region = None
for s_region in surf_components:
    surf_region = BoolSurface.mult(surf_region, s_region + (-plane_region))

surf_region.region.simplify(None)
reversedCC_region = plane_region * surf_region
```

### `PlaneSeq` — el grupo de planos de unión

`build_RCC_params` (`functions.py`) recoge el plano de unión de cada pieza en una
**lista plana** y llama a `convex_planes(group_planes, cylcones[0].Surf.Axis)`
**una vez para todo el grupo**: construye `PlaneSeq` como un OR-bracket
(`orientation == "Forward"`) o una lista AND plana (`"Reversed"`).

> **⚠** Antes, el operador AND/OR entre piezas adyacentes se decidía
> *incrementalmente, por conexión*, vía
> `operator = "AND" if d · adjPlane.Axis > 0 else "OR"`, donde `d` venía de la
> **dirección de traversal del parámetro UV de la cara semilla** — una
> referencia **no invariante** (la dirección de parametrización de una cara es
> implementation-defined del kernel CAD, no consistente entre instancias físicas
> del mismo rasgo). El campo `.Connections` y el parámetro `parent_id` que
> soportaban ese cálculo se eliminaron.
> Wraparound en el bucle `emin`/`emax` de `get_join_cone_cyl`:
> `d = min(d, twoPi - d)` y reducir `u` vía `twoPimod(u)` antes de calcular `d`.

### `AddPlanes` (planos adicionales de cierre)

El bucle que construye `plane_region` desde `reversedCC.Surf.AddPlanes` usaba
`BoolSurface.add` (OR) **incondicional** → cambiado a `BoolSurface.mult` (AND).
*"La combinación del RevCC con el plano tiene que ser AND."* La lógica
Forward/Reversed de `PlaneSeq` no se toca — esto era específico de que
`AddPlanes` siempre defaulteaba a OR.

### `AdjacentMultiplanePlanes` (planos de un MultiPlane que bordea el RevCC)

Ahora **lista de listas** — una sublista por grupo `MultiPlane` distinto.
Construida en `get_join_cone_cyl` llamando a `_find_adjacent_multiplane_planes`
**una vez por candidato `MultiPlane`** (antes: una vez por segmento, aplanando
todo). `build_RCC_params` ya no deduplica ni fusiona entre grupos.

Región (`geouned_classes.py`):

- por grupo: **AND** de los planos negados de ese grupo (`multiplane_region` =
  estar fuera de *todas* las facetas de ese grupo a la vez);
- **OR** entre los `multiplane_region` de grupos distintos
  (`multiplane_set_region` = estar fuera de *cualquier* grupo entero basta);
- **OR** de eso en `reversedCC_region`.

**AND dentro de grupo, OR entre grupos, OR en la región principal.**

- `_find_adjacent_multiplane_planes` (`meta_surfaces_utils.py`) hace una
  búsqueda **invertida**: recorre *cada* arista del shell del segmento y
  comprueba si su vecino real (vía `other_face_edge`, tolerante a slivers) ya es
  uno de los planos-componente de los `multiplanes` conocidos — en vez de
  adivinar por forma/posición (que fallaba con rims elípticos de cortes no
  perpendiculares). Threading de las `tolerances` reales, no `Tolerances()` por
  defecto.
- Acepta `axial_bounds=(vmin, vmax)`: solo acepta una arista curva cuyo parámetro
  V muestreado cae cerca de uno de los dos **extremos axiales reales** del
  segmento — evita tomar un plano real a media altura (p. ej. Z=4000 dentro de un
  segmento Z∈[−1000, 6500]) como si fuera un cierre.

### Verificación de `check_sign` (lado de conversión)

Estructuralmente inalcanzable desde descomposición (RevCC no corta). Se verificó
desde el lado de conversión: `_reversedCC_component` devuelve una 3-tupla
`(s_region, p_region, components)`; `check_sign` con una rama
`"ReversedConeCylinder"` como Can/TCone; muestreo en una caja local derivada de
las caras reales que matchean cada segmento (no la del sólido entero, que da
falsos negativos por términos de contorno ajenos a la fórmula del RevCC).

- `cyl_cone.stp`: **300/300**.
- `hylife-v06` solid113: **271/300** — discrepancia *real* (sobre-predicción de
  material en la banda Z ≈ [4050,8 , 4396,5]), ligada al desajuste conocido del
  round-trip de GEOReverse, **no raíz-causada**.

### Pendientes relacionados (**⚠**)

- Extender `AdjacentMultiplanePlanes` de RevCC a **MultiRoundCorner**.
- `gen_plane_cylinder` / `gen_plane_cone` (`meta_surfaces_utils.py`) operan sobre
  una cara cruda (`Faces[ifacemin]`/`Faces[ifacemax]`), no sobre un shell
  fusionado, cuando el cilindro/cono de un segmento está partido en varias piezas
  contiguas — gap arquitectónico real (el mismo patrón ya se arregló una vez en
  `cyl_plane_region_conf`); sin caso de reproducción por ahora.
- `gen_plane_cone` / `gen_plane_cylinder` computan la normal del plano **sin
  corrección de dirección-de-material** (a diferencia de `find_can_plane` /
  `cks_edge_plane`, que aplican `material_direction`) — defecto real separado, no
  arreglado.
- `add_reversedCC` no popula `.components` en producción (el scaffolding de
  verificación se revirtió); si se quiere `check_sign` sobre RevCC de forma
  permanente, hay que reintroducirlo.

---

## 7. Convenciones transversales y trampas de método

- **Rechazar un candidato de superficie en tiempo de descomposición puede
  desviar silenciosamente `generic_split` por otra ruta de corte** y romper una
  cara no relacionada en el mismo sólido. Un cambio que parece un estrechamiento
  estricto **no es local**. Precedentes concretos: `get_can_surfaces` /
  `outer2_only` (la intuición sobre "más estricto = más seguro" falló dos veces
  seguidas en esa función); `_plane_recrosses_arc` (revertido).
- **Todo cambio en esta área debe re-ejecutar el escaneo diferencial de corpus
  completo** (`Solidos/test_models`, cuentas
  Can/TCone/RoundCorner/MultiRoundCorner/MultiPlane/RevCC, `git stash`
  antes/después), no razonarse en abstracto. Idealmente seguido de un chequeo
  d1suned de volumen sobre los ficheros que cambian.
- **Dos configuraciones genuinamente distintas no pueden dar la misma expresión
  booleana.** Si `(AND_p1_cyl, AND_p2_cyl, AND_p1_pd, AND_p2_pd)` colapsan a la
  misma fórmula para combinaciones distintas, la **clasificación** está mal, no
  la fórmula.
- Encuestas estáticas de "quién lee este campo" no bastan cuando el valor pasa
  por un objeto intermedio — seguir cada `AttributeError`/`TypeError` real de la
  suite completa.
- `check_sign` sobre RevCC nunca hace falta durante descomposición — que no
  exista esa rama no es un descuido.
- Verificación de `check_sign` punto a punto: siempre muestrear **dentro de la
  caja de construcción** del `make*`, no del BoundBox del sólido resultante +
  padding (los constructores ya extienden un poco su forma más allá de la caja;
  padear encima empuja puntos fuera de donde el sólido de referencia tiene
  geometría).

---

## 8. Referencias externas (no en el repo / no accesibles ahora)

- **`configuracion_cans.txt`** — especificación completa y autoritativa de las
  reglas AND/OR de Can, escrita por el usuario. Ubicación:
  `C:\Users\Patrick\Work\Taller\GEOUNED_workshop\configuracion_cans.txt` (otro
  equipo). §2 de este documento solo resume lo que `CLAUDE.md` reproduce de ella
  — la 2×2 grid, el caso "boca abierta", la regla cono+plano-ápice.
- **`composite_surface_definitions.md`** — referenciado en `CLAUDE.md` como el
  documento con la "definición teórica" y su "sección RoundCorner" (incluidas las
  "4 configuraciones"). No presente en la carpeta `memory/` de este equipo. Este
  documento pretende ser su sustituto y consolidación; en fase 2 conviene
  cotejarlos y unificarlos.

---

## Mantenimiento

- Este fichero es la **referencia de estado actual**. La historia detallada de
  cada bug (intentos, reversiones, verificaciones fichero a fichero) vive en
  `CLAUDE.md` y no debe duplicarse aquí — aquí solo la conclusión y la regla que
  queda.
- Fase 2 (con acceso al otro equipo): resolver cada **⚠ verificar**, cotejar con
  `configuracion_cans.txt` y `composite_surface_definitions.md`, e incorporar
  cualquier sesión posterior al commit `dcb774e`.
