from tortoise import fields, models

class Playbook(models.Model):
    id = fields.IntField(pk=True)
    name=fields.StringField(max_length=255)
    created_at = fields.DatetimeField(auto_now_add=True)
    updated_at = fields.DatetimeField(auto_now=True)

    class Meta:
        table = "playbooks"