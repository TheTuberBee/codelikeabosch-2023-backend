"""
Database model storing previously analyzed demos.
"""

from tortoise import fields, models


class Demo(models.Model):
    id = fields.IntField(pk=True)
    name = fields.CharField(max_length=255)
    data = fields.JSONField()
    created_at = fields.DatetimeField(auto_now_add=True)

    class Meta:
        table = "demos"
