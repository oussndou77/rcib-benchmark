# Guide RunPod — Phase 0 (smoke test CARLA) — VALIDÉ

Ce guide reflète le chemin réel validé en Phase 0 (et non la théorie). L'image
CARLA brute sur RunPod réserve plusieurs pièges ; ils sont tous documentés ici.

## Résumé du chemin qui marche

1. Créer le pod avec l'image `carlasim/carla:0.9.15`, start command `bash -c "sleep infinity"`
2. Se connecter en **SSH** (PAS le web terminal — il ne marche pas avec cette image)
3. Lancer `setup_runpod.sh` (installe les libs sans root) puis `source ~/rcib_env.sh`
4. Lancer `launch_carla.sh` (serveur + smoke test)
5. Éteindre le pod dès que c'est validé

---

## 1. Créer le pod

- Image : `carlasim/carla:0.9.15`
- GPU : RTX 3090 (ou A4000/A5000/L4) — supporte Vulkan, abordable
- Container disk : **35 GB minimum**
- Start command : `bash -c "sleep infinity"`
- Variable d'env : `NVIDIA_DRIVER_CAPABILITIES=all`
- ⚠️ Le premier déploiement télécharge ~20 GB (l'image est lourde) : compte 3-5 min.
  ATTENDS que TOUS les layers soient "Pull complete" avant de te connecter.

## 2. Se connecter (SSH, pas le web terminal)

**Le web terminal RunPod ne fonctionne PAS avec l'image CARLA** (il attend la
plomberie des images officielles RunPod). On utilise le SSH proxy.

Prérequis (une seule fois) : générer une clé SSH et l'ajouter dans les Settings RunPod.
```bash
ls ~/.ssh/id_ed25519.pub || ssh-keygen -t ed25519 -C "ton@email" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub      # → copier dans RunPod Settings > SSH Public Keys
```
Puis, onglet Connect du pod → copier la commande SSH proxy (te connecte en `carla`) :
```bash
ssh <id>@ssh.runpod.io -i ~/.ssh/id_ed25519
```

## 3. Setup de l'environnement (sans root)

L'image tourne en utilisateur `carla` SANS root, Python 3.6, et il manque 3 libs
système. On ne peut pas `apt install`, mais on peut télécharger les .deb et les
extraire localement. Le script automatise tout :
```bash
# git n'est pas dans l'image -> télécharger l'archive du repo
cd ~
python3 -c "import urllib.request; urllib.request.urlretrieve('https://github.com/oussndou77/rcib-benchmark/archive/refs/heads/main.tar.gz','rcib.tar.gz')"
tar xzf rcib.tar.gz          # crée rcib-benchmark-main/
cd rcib-benchmark-main/runners
bash setup_runpod.sh          # installe libs + egg + dataclasses, écrit ~/rcib_env.sh
source ~/rcib_env.sh
```
Le script télécharge `libjpeg-turbo8`, `libtiff5`, `libjbig0`, les extrait dans
`~/locallibs`, localise l'egg carla py3.7, écrit `~/rcib_env.sh` (PYTHONPATH +
LD_LIBRARY_PATH + LC_ALL), et teste l'import.

## 4. Lancer le serveur + smoke test

```bash
bash launch_carla.sh          # -RenderOffScreen par défaut
# si crash Vulkan :
bash launch_carla.sh --nullrhi
```
Résultat attendu : `CONNEXION REUSSIE`, versions client/serveur = 0.9.15.

⚠️ CARLA est LENT à démarrer (~60-90s). Ne pas confondre "lent" et "planté".
Le script attend 90s par défaut.

## 5. Éteindre le pod

Dès que le smoke test passe → **Stop** le pod (le compteur tourne).

---

## Pièges rencontrés et solutions (mémo Phase 0)

| Piège | Cause | Solution |
|-------|-------|----------|
| Web terminal retombe sur "Stopped" | image CARLA sans plomberie RunPod | utiliser SSH |
| `apt install` → Permission denied | utilisateur `carla`, pas de root | `apt-get download` + `dpkg -x` (sans root) |
| wheel `cp37` ne s'installe pas | Python du conteneur = 3.6, pas 3.7 | utiliser l'**egg** py3.7 (tolère 3.6) via PYTHONPATH |
| `ImportError: libjpeg.so.8` etc. | 3 libs système absentes de l'image | les télécharger en .deb et extraire dans ~/locallibs |
| `UnicodeEncodeError surrogates` | encodage SSH/Windows | `export LC_ALL=C.UTF-8` |
| serveur `-RenderOffScreen` "meurt" | en fait LENT à démarrer (pas mort) | attendre 90s ; vérifier `ps aux \| grep Shipping` |
| `bind: Address already in use` | deux serveurs sur le port 2000 | `pkill -9 -f CarlaUE4` avant de relancer |
| `git: command not found` | image minimale sans git | télécharger le repo en archive : `python3 -c "import urllib.request; urllib.request.urlretrieve('https://github.com/USER/REPO/archive/refs/heads/main.tar.gz','r.tar.gz')"` puis `tar xzf` |
| `No module named 'dataclasses'` | Python 3.6 (dataclasses = stdlib 3.7+) | `setup_runpod.sh` récupère le backport automatiquement (un fichier pur Python) |

## La vraie solution durable (à faire à froid)

Le container disk est effacé à chaque arrêt → il faut relancer setup_runpod.sh à
chaque pod. La solution "production" est de construire une image Docker custom
(voir docker/Dockerfile.runpod) qui embarque les libs + l'egg + le code, et de la
pousser sur Docker Hub. Les pods démarreront alors en ~30s sans bricolage.
