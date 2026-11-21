"""The planner's contributors for this drive, chosen from the saved config. Each is a callable(ctx) ->
planner.Contribution or None. Stage 2 (c) has none: with them absent nothing is ever pressed."""


def build(cfg, CP, log=None):
  return []
