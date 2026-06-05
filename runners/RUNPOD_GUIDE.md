# Guide RunPod — Phase 0 (smoke test CARLA)

Objectif : vérifier sur un GPU RunPod que CARLA 0.9.15 démarre headless et que le
client Python s'y connecte (sans mismatch de version). Court et peu coûteux si on
suit l'ordre : on prépare tout, on lance le pod, on teste, on éteint.

---

## Avant de lancer le pod (gratuit — à faire à froid)

Tu as déjà les 3 fichiers de ce dossier `runners/` + `docker/` :
- `runners/smoke_test.py` — le test de connexion
- `runners/launch_carla.sh` — lance le serveur + le test
- `docker/Dockerfile.runpod` — (pour plus tard, Phase 2)

Pour la Phase 0, **pas besoin de builder le Dockerfile** : on lance directement
l'image officielle CARLA sur RunPod et on copie nos 2 scripts dedans.

---

## Étape 1 — Créer le pod sur RunPod

1. Console RunPod → **Deploy** / **+ GPU Pod**
2. **GPU** : choisis un GPU abordable avec assez de VRAM. **RTX 3090, A4000, A5000 ou
   L4** conviennent (≥ 12 Go VRAM recommandé). Pas besoin d'A100 — CARLA fait du
   rendu, pas de l'entraînement.
3. **Template / Image** : dans le champ image Docker custom, mets :
   ```
   carlasim/carla:0.9.15
   ```
4. **Container Disk** : **au moins 30 Go** (l'image CARLA est lourde).
5. **Container Start Command** : c'est le point délicat. L'image CARLA démarre par
   défaut le serveur, ce qui nous empêcherait d'avoir un shell. On veut un shell pour
   piloter. Mets une commande qui garde le conteneur vivant ET nous donne un accès :
   ```
   bash -c "sleep infinity"
   ```
   (On lancera le serveur CARLA nous-mêmes via `launch_carla.sh`.)
6. **Ports** : expose le port **22/tcp** (SSH) si tu veux SSH. Le web terminal RunPod
   marche aussi sans ça.
7. **Deploy**. Attends que le pod passe en "Running".

---

## Étape 2 — Se connecter au pod

Via le **web terminal** RunPod (bouton "Connect" → "Start Web Terminal") ou en SSH
si tu l'as configuré. Tu obtiens un shell **root** dans le conteneur.

Vérifie d'abord que le GPU est bien là :
```bash
nvidia-smi
```
Tu dois voir ton GPU. **Si cette commande échoue, CARLA ne marchera pas** — arrête-toi
et vérifie la config du pod (runtime NVIDIA).

---

## Étape 3 — Installer pip + le wheel carla + copier nos scripts

CARLA tourne sous l'utilisateur `carla`, le code est dans `/home/carla`.

```bash
# pip si absent
apt-get update && apt-get install -y python3-pip xdg-user-dirs

# Installer le package carla DEPUIS l'image (garantit version client==serveur)
WHEEL=$(ls /home/carla/PythonAPI/carla/dist/carla-0.9.15-*.whl | head -n1)
echo "Wheel trouvé : $WHEEL"
python3 -m pip install "$WHEEL"
```

Puis récupère nos 2 scripts. Le plus simple : clone ton repo GitHub (la Phase 1 y est
déjà ; ajoute-y `runners/` au prochain push), ou crée les fichiers à la main via
`nano`. Si tu as poussé `runners/` sur GitHub :
```bash
cd /home/carla
git clone https://github.com/oussndou77/rcib-benchmark.git
cd rcib-benchmark/runners
```

---

## Étape 4 — Lancer le smoke test

```bash
bash launch_carla.sh
```

Ce script :
1. démarre le serveur CARLA en headless (`-RenderOffScreen -nosound`)
2. attend ~45 s qu'il soit prêt
3. lance `smoke_test.py` qui se connecte, vérifie la version, fait des ticks, spawn un véhicule

**Résultat attendu** : une série de ✓ se terminant par
```
✓✓✓ SMOKE TEST RÉUSSI — CARLA + client + GPU opérationnels.
```

---

## Étape 5 — Éteindre le pod (important pour le coût)

Dès que le test passe, **arrête le pod** depuis la console RunPod (Stop/Terminate).
Tu ne paies que le temps réellement utilisé. La Phase 0 doit prendre 15-30 min max.

---

## Dépannage (les erreurs probables et leur cause)

| Symptôme | Cause | Solution |
|----------|-------|----------|
| `nvidia-smi` échoue | pas de runtime GPU | recréer le pod avec un template NVIDIA |
| serveur s'arrête tout de suite | GPU/Vulkan indispo | vérifier `nvidia-smi`, essayer un autre type de GPU |
| `package carla non importable` | wheel pas installé | refaire l'étape 3 (install du wheel) |
| **MISMATCH de version** | client ≠ serveur | le test te donne la commande exacte de réinstall du bon wheel |
| connexion timeout | serveur pas encore prêt | augmenter `WAIT_SECONDS` (ex. `WAIT_SECONDS=90 bash launch_carla.sh`) |
| `xdg-user-dir: not found` | paquet manquant | `apt-get install -y xdg-user-dirs` (déjà dans l'étape 3) |

---

## Pourquoi ce test, et ce qu'il prouve

Si le smoke test passe, on a validé la fondation la plus risquée du projet : que CARLA
tourne sur le GPU cloud, en headless, et que notre client Python communique avec lui
**à la bonne version**. Tout le Scenario Bridge (Phase 2) et l'Ego Planner (Phase 3)
se construisent sur cette connexion. On ne code pas la suite tant que ce ✓ n'est pas
obtenu — c'est ce qui évite de débugger 5 problèmes à la fois.
