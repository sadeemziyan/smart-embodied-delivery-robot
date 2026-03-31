![Genesis](imgs/big_text.png)

![Teaser](imgs/teaser.png)

[![PyPI - Version](https://img.shields.io/pypi/v/genesis-world)](https://pypi.org/project/genesis-world/)
[![PyPI - Downloads](https://img.shields.io/pypi/dm/genesis-world)](https://pypi.org/project/genesis-world/)
[![GitHub Issues](https://img.shields.io/github/issues/Genesis-Embodied-AI/Genesis)](https://github.com/Genesis-Embodied-AI/Genesis/issues)
[![GitHub Discussions](https://img.shields.io/github/discussions/Genesis-Embodied-AI/Genesis)](https://github.com/Genesis-Embodied-AI/Genesis/discussions)
[![Discord](https://img.shields.io/discord/1322086972302430269?logo=discord)](https://discord.gg/nukCuhB47p)
<a href="https://drive.google.com/uc?export=view&id=1ZS9nnbQ-t1IwkzJlENBYqYIIOOZhXuBZ"><img src="https://img.shields.io/badge/WeChat-07C160?style=for-the-badge&logo=wechat&logoColor=white" height="20" style="display:inline"></a>

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![README en Français](https://img.shields.io/badge/Francais-d9d9d9)](./README_FR.md)
[![한국어 README](https://img.shields.io/badge/한국어-d9d9d9)](./README_KR.md)
[![简体中文版自述文件](https://img.shields.io/badge/简体中文-d9d9d9)](./README_CN.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

# Genesis

## 🔥 Nouveautés

  - [2025-08-05] Sortie de la v0.3.0 🎊 🎉
  - [2025-07-02] Le développement de Genesis est désormais officiellement soutenu par [Genesis AI](https://genesis-ai.company/).
  - [2025-01-09] Nous avons publié un [rapport détaillé d'analyse comparative des performances](https://github.com/zhouxian/genesis-speed-benchmark) de Genesis, accompagné de tous les scripts de test.
  - [2025-01-08] Sortie de la v0.2.1 🎊 🎉
  - [2025-01-08] Création des groupes [Discord](https://discord.gg/nukCuhB47p) et [Wechat](https://drive.google.com/uc?export=view&id=1ZS9nnbQ-t1IwkzJlENBYqYIIOOZhXuBZ).
  - [2024-12-25] Ajout d’un [docker](https://www.google.com/search?q=%23docker) incluant la prise en charge du moteur de rendu par lancer de rayon (ray-tracing).
  - [2024-12-24] Ajout de directives pour [contribuer à Genesis](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/.github/contributing/PULL_REQUESTS.md).

## Table des Matières

1. [Qu'est-ce que Genesis ?](#quest-ce-que-genesis-)
2. [Caractéristiques clés](#principales-caract%C3%A9ristiques)
3. [Installation Rapide](#installation-rapide)
4. [Docker](#docker)
5. [Documentation](#documentation)
6. [Contribuer à Genesis](#contribution-%C3%A0-genesis)
7. [Support](#support)
8. [License et Remerciements](#licence-et-remerciements)
9. [Articles Associés](#publications-associ%C3%A9es)
10. [Citation](#citation)

## Qu'est-ce que Genesis ?

Genesis est une plateforme physique conçue pour des applications générales en *Robotique/ IA embarquée/IA physique*. Elle combine plusieurs fonctionnalités :

1. Un **moteur physique universel**, reconstruit depuis zéro, capable de simuler une large gamme de matériaux et de phénomènes physiques.
2. Une plateforme de simulation robotique **légère**, **ultra-rapide**,**pythonic**, et **conviviale**.
3. Un puissant et rapide **système de rendu photo-réaliste**.
4. Un **moteur de génération de données** qui transforme des descriptions en langage naturel en divers types de données.

Genesis vise à :

- **Réduire les barrières** à l'utilisation des simulations physiques, rendant la recherche en robotique accessible à tous. Voir notre [déclaration de mission](https://genesis-world.readthedocs.io/en/latest/user_guide/overview/mission.html).
- **Unifier divers solveurs physiques** dans un cadre unique pour recréer le monde physique avec la plus haute fidélité.
- **Automatiser la génération de données**, réduisant l'effort humain et permettant à l'écosystème de données de fonctionner de manière autonome.

Page du projet : <https://genesis-embodied-ai.github.io/>

## Principales Caractéristiques

- **Vitesse** : Plus de 43 millions d'IPS lors de la simulation d'un bras robotique Franka avec une seule RTX 4090 (430 000 fois plus rapide que le temps réel).
- **Multi-plateforme** : Fonctionne sur Linux, macOS, Windows, et prend en charge plusieurs backends de calcul (CPU, GPU Nvidia/AMD, Apple Metal).
- **Intégration de divers solveurs physiques** : Corps rigides, MPM, SPH, FEM, PBD, Fluides stables.
- **Large éventail de modèles de matériaux** : Simulation et couplage de corps rigides, liquides, gaz, objets déformables, objets à coque mince et matériaux granulaires.
- **Compatibilité avec divers robots** : Bras robotiques, robots à pattes, drones, *robots mous*, et support pour charger `MJCF (.xml)`, `URDF`, `.obj`, `.glb`, `.ply`, `.stl`, et plus encore.
- **Rendu photo-réaliste** : Rendu natif basé sur le lancer de rayons.
- **Différentiabilité** : Genesis est conçu pour être entièrement différentiable. Actuellement, notre solveur MPM et Tool Solver prennent en charge la différentiabilité, avec d'autres solveurs prévus dans les prochaines versions (à commencer par le solveur de corps rigides et articulés).
- **Facilité d'utilisation** : Conçu pour être simple, avec une installation intuitive et des API conviviales.

## Installation Rapide

Installez d'abord **PyTorch** en suivant les [instructions officielles](https://pytorch.org/get-started/locally/).

Ensuite, installez Genesis via PyPI :

```bash
pip install genesis-world  # Nécessite Python>=3.10,<3.14;
```

Pour obtenir la version la plus récente, assurez-vous que `pip` est à jour via `pip install --upgrade pip`, puis exécutez la commande :

```bash
pip install git+https://github.com/Genesis-Embodied-AI/Genesis.git
```

Notez que le paquet doit toujours être mis à jour manuellement pour se synchroniser avec la branche principale (main).

Les utilisateurs souhaitant modifier le code source de Genesis sont encouragés à l'installer en mode éditable. D'abord, assurez-vous que `genesis-world` a été désinstallé, puis clonez le dépôt et installez-le localement :

```bash
git clone https://github.com/Genesis-Embodied-AI/Genesis.git
cd Genesis
pip install -e ".[dev]"
```
Il est recommandé d'exécuter systématiquement `pip install -e ".[dev]"` après avoir déplacé le HEAD pour s'assurer que toutes les dépendances et points d'entrée sont à jour.

### Utiliser uv

[uv](https://docs.astral.sh/uv/) est un gestionnaire de paquets et de projets Python rapide.

**Installer uv :**
```bash
# Sur macOS et Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Sur Windows
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**Démarrage rapide avec uv :**
```bash
git clone https://github.com/Genesis-Embodied-AI/Genesis.git
cd Genesis
uv sync
```

Ensuite, installez PyTorch pour votre plateforme :

```bash
# GPU NVIDIA (CUDA 12.6 par exemple)
uv pip install torch --index-url https://download.pytorch.org/whl/cu126

# CPU uniquement (Linux/Windows)
uv pip install torch --index-url https://download.pytorch.org/whl/cpu

# Apple Silicon (Metal/MPS)
uv pip install torch
```

Exécutez un exemple :
```bash
uv run examples/rigid/single_franka.py
```

## Docker

Si vous souhaitez utiliser Genesis depuis Docker, vous pouvez d'abord construire l'image Docker comme suit :

```bash
docker build -t genesis -f docker/Dockerfile docker
```

Vous pouvez ensuite exécuter les exemples à l'intérieur de l'image Docker (montés dans `/workspace/examples`) :

```bash
xhost +local:root # Autoriser le conteneur à accéder à l'affichage

docker run --gpus all --rm -it \
-e DISPLAY=$DISPLAY \
-e LOCAL_USER_ID="$(id -u)" \
-v /dev/dri:/dev/dri \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
-v $(pwd):/workspace \
--name genesis genesis:latest
```

### Utilisateurs AMD

Les utilisateurs AMD peuvent utiliser Genesis avec le fichier `docker/Dockerfile.amdgpu`, qui se construit en exécutant :

```
docker build -t genesis-amd -f docker/Dockerfile.amdgpu docker
```

et peut ensuite être utilisé en exécutant :

```xhost +local:docker \
docker run -it --network=host \
 --device=/dev/kfd \
 --device=/dev/dri \
 --group-add=video \
 --ipc=host \
 --cap-add=SYS_PTRACE \
 --security-opt seccomp=unconfined \
 --shm-size 8G \
 -v $PWD:/workspace \
 -e DISPLAY=$DISPLAY \
 genesis-amd
```

Les exemples seront accessibles depuis `/workspace/examples`. Note : Les utilisateurs AMD doivent utiliser le backend ROCm (HIP). Cela signifie que vous devrez appeler `gs.init(backend=gs.amdgpu)` pour initialiser Genesis.

## Documentation

Une documentation complète est disponible en [Anglais](https://genesis-world.readthedocs.io/en/latest/user_guide/index.html) et en [Chinois](https://genesis-world.readthedocs.io/zh-cn/latest/user_guide/index.html). Cela inclut des étapes d'installation détaillées, des tutoriels et des références API.

## Contribution à Genesis

Le projet Genesis est un effort ouvert et collaboratif. Nous accueillons toutes les formes de contributions de la communauté, notamment :

- **Pull requests** pour de nouvelles fonctionnalités ou des corrections de bugs.
- **Rapports de bugs** via GitHub Issues.
- **Suggestions** pour améliorer la convivialité de Genesis.

Consultez notre [guide de contribution](https://github.com/Genesis-Embodied-AI/Genesis/blob/main/.github/contributing/PULL_REQUESTS.md) pour plus de détails.

## Support

- Signalez des bugs ou demandez des fonctionnalités via GitHub [Issues](https://github.com/Genesis-Embodied-AI/Genesis/issues).
- Participez aux discussions ou posez des questions sur GitHub [Discussions](https://github.com/Genesis-Embodied-AI/Genesis/discussions).

## Licence et Remerciements

Le code source de Genesis est sous licence Apache 2.0.

Le développement de Genesis a été rendu possible grâce à ces projets open-source :

- [Taichi](https://github.com/taichi-dev/taichi) : Backend de calcul multiplateforme haute performance. Merci à l'équipe de Taichi pour leur support technique !
- [FluidLab](https://github.com/zhouxian/FluidLab) : Implémentation de référence du solveur MPM.
- [SPH_Taichi](https://github.com/erizmr/SPH_Taichi) : Implémentation de référence du solveur SPH.
- [Ten Minute Physics](https://matthias-research.github.io/pages/tenMinutePhysics/index.html) et [PBF3D](https://github.com/WASD4959/PBF3D) : Implémentations de référence des solveurs PBD.
- [MuJoCo](https://github.com/google-deepmind/mujoco) : Référence pour la dynamique des corps rigides.
- [libccd](https://github.com/danfis/libccd) : Référence pour la détection des collisions.
- [PyRender](https://github.com/mmatl/pyrender) : Rendu basé sur la rasterisation.
- [LuisaCompute](https://github.com/LuisaGroup/LuisaCompute) et [LuisaRender](https://github.com/LuisaGroup/LuisaRender) : DSL de ray-tracing.

## Publications Associées

Genesis est un projet à grande échelle qui intègre des technologies de pointe issues de divers travaux de recherche existants et en cours dans un seul système. Voici une liste non exhaustive de toutes les publications qui ont contribué au projet Genesis d'une manière ou d'une autre :

- Xian, Zhou, et al. "Fluidlab: A differentiable environment for benchmarking complex fluid manipulation." arXiv preprint arXiv:2303.02346 (2023).
- Xu, Zhenjia, et al. "Roboninja: Learning an adaptive cutting policy for multi-material objects." arXiv preprint arXiv:2302.11553 (2023).
- Wang, Yufei, et al. "Robogen: Towards unleashing infinite data for automated robot learning via generative simulation." arXiv preprint arXiv:2311.01455 (2023).
- Wang, Tsun-Hsuan, et al. "Softzoo: A soft robot co-design benchmark for locomotion in diverse environments." arXiv preprint arXiv:2303.09555 (2023).
- Wang, Tsun-Hsuan Johnson, et al. "Diffusebot: Breeding soft robots with physics-augmented generative diffusion models." Advances in Neural Information Processing Systems 36 (2023): 44398-44423.
- Katara, Pushkal, Zhou Xian, and Katerina Fragkiadaki. "Gen2sim: Scaling up robot learning in simulation with generative models." 2024 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2024.
- Si, Zilin, et al. "DiffTactile: A Physics-based Differentiable Tactile Simulator for Contact-rich Robotic Manipulation." arXiv preprint arXiv:2403.08716 (2024).
- Wang, Yian, et al. "Thin-Shell Object Manipulations With Differentiable Physics Simulations." arXiv preprint arXiv:2404.00451 (2024).
- Lin, Chunru, et al. "UBSoft: A Simulation Platform for Robotic Skill Learning in Unbounded Soft Environments." arXiv preprint arXiv:2411.12711 (2024).
- Zhou, Wenyang, et al. "EMDM: Efficient motion diffusion model for fast and high-quality motion generation." European Conference on Computer Vision. Springer, Cham, 2025.
- Qiao, Yi-Ling, Junbang Liang, Vladlen Koltun, and Ming C. Lin. "Scalable differentiable physics for learning and control." International Conference on Machine Learning. PMLR, 2020.
- Qiao, Yi-Ling, Junbang Liang, Vladlen Koltun, and Ming C. Lin. "Efficient differentiable simulation of articulated bodies." In International Conference on Machine Learning, PMLR, 2021.
- Qiao, Yi-Ling, Junbang Liang, Vladlen Koltun, and Ming Lin. "Differentiable simulation of soft multi-body systems." Advances in Neural Information Processing Systems 34 (2021).
- Wan, Weilin, et al. "Tlcontrol: Trajectory and language control for human motion synthesis." arXiv preprint arXiv:2311.17135 (2023).
- Wang, Yian, et al. "Architect: Generating Vivid and Interactive 3D Scenes with Hierarchical 2D Inpainting." arXiv preprint arXiv:2411.09823 (2024).
- Zheng, Shaokun, et al. "LuisaRender: A high-performance rendering framework with layered and unified interfaces on stream architectures." ACM Transactions on Graphics (TOG) 41.6 (2022): 1-19.
- Fan, Yingruo, et al. "Faceformer: Speech-driven 3d facial animation with transformers." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2022.
- Wu, Sichun, Kazi Injamamul Haque, and Zerrin Yumak. "ProbTalk3D: Non-Deterministic Emotion Controllable Speech-Driven 3D Facial Animation Synthesis Using VQ-VAE." Proceedings of the 17th ACM SIGGRAPH Conference on Motion, Interaction, and Games. 2024.
- Dou, Zhiyang, et al. "C· ase: Learning conditional adversarial skill embeddings for physics-based characters." SIGGRAPH Asia 2023 Conference Papers. 2023.

... et bien d'autres travaux en cours.

## Citation

Si vous utilisez Genesis dans vos recherches, veuillez envisager de citer :

```bibtex
@misc{Genesis,
  author = {Genesis Authors},
  title = {Genesis: A Generative and Universal Physics Engine for Robotics and Beyond},
  month = {December},
  year = {2024},
  url = {https://github.com/Genesis-Embodied-AI/Genesis}
}
```