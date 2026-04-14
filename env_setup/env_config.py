class EnvConfig:
    def __init__(
        self,
        floor_size=(10, 10),
        floor_complexity=None,
        cell_size=0.5,
        wall_height=0.4,
        floor_thickness=0.05,
        num_floors=1,
        show_viewer=False,
    ):
        self.floor_size       = floor_size        # (rows, cols)
        self.floor_complexity = floor_complexity  # TBD
        self.cell_size        = cell_size
        self.wall_height      = wall_height
        self.floor_thickness  = floor_thickness
        self.num_floors       = num_floors
        self.show_viewer      = show_viewer